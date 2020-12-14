// SPDX-License-Identifier: GPL-2.0+
/* Microchip Sparx5 Switch driver
 *
 * Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
 */

#include "sparx5_main_regs.h"
#include "sparx5_main.h"

#define XTR_EOF_0     ntohl((__force __be32)0x80000000u)
#define XTR_EOF_1     ntohl((__force __be32)0x80000001u)
#define XTR_EOF_2     ntohl((__force __be32)0x80000002u)
#define XTR_EOF_3     ntohl((__force __be32)0x80000003u)
#define XTR_PRUNED    ntohl((__force __be32)0x80000004u)
#define XTR_ABORT     ntohl((__force __be32)0x80000005u)
#define XTR_ESCAPE    ntohl((__force __be32)0x80000006u)
#define XTR_NOT_READY ntohl((__force __be32)0x80000007u)

#define XTR_VALID_BYTES(x)      (4 - ((x) & 3))

#define XTR_QUEUE     0
#define INJ_QUEUE     0

struct frame_info {
	int src_port;
};

static void sparx5_xtr_flush(struct sparx5 *sparx5, u8 grp)
{
	/* Start flush */
	spx5_wr(QS_XTR_FLUSH_FLUSH_SET(BIT(grp)), sparx5, QS_XTR_FLUSH);

	/* Allow to drain */
	mdelay(1);

	/* All Queues normal */
	spx5_wr(0, sparx5, QS_XTR_FLUSH);
}

static void sparx5_ifh_parse(u32 *ifh, struct frame_info *info)
{
	u8 *xtr_hdr = (u8 *)ifh;

	/* FWD is bit 45-72 (28 bits), but we only read the 27 LSB for now */
	u32 fwd =
		((u32)xtr_hdr[27] << 24) |
		((u32)xtr_hdr[28] << 16) |
		((u32)xtr_hdr[29] <<  8) |
		((u32)xtr_hdr[30] <<  0);
	fwd = (fwd >> 5);
	info->src_port = FIELD_GET(GENMASK(7, 1), fwd);
}

static void sparx5_xtr_grp(struct sparx5 *sparx5, u8 grp, bool byte_swap)
{
	int i, byte_cnt = 0;
	bool eof_flag = false, pruned_flag = false, abort_flag = false;
	u32 ifh[IFH_LEN];
	struct sk_buff *skb;
	struct frame_info fi;
	struct sparx5_port *port;
	struct net_device *netdev;
	u32 *rxbuf;

	/* Get IFH */
	for (i = 0; i < IFH_LEN; i++)
		ifh[i] = spx5_rd(sparx5, QS_XTR_RD(grp));

	/* Decode IFH (whats needed) */
	sparx5_ifh_parse(ifh, &fi);

	/* Map to port netdev */
	port = fi.src_port < SPX5_PORTS ?
		sparx5->ports[fi.src_port] : NULL;
	if (!port || !port->ndev) {
		dev_err(sparx5->dev, "Data on inactive port %d\n", fi.src_port);
		sparx5_xtr_flush(sparx5, grp);
		return;
	}

	/* Have netdev, get skb */
	netdev = port->ndev;
	skb = netdev_alloc_skb(netdev, netdev->mtu + ETH_HLEN);
	if (!skb) {
		sparx5_xtr_flush(sparx5, grp);
		dev_err(sparx5->dev, "No skb allocated\n");
		return;
	}
	rxbuf = (u32 *)skb->data;

	/* Now, pull frame data */
	while (!eof_flag) {
		u32 val = spx5_rd(sparx5, QS_XTR_RD(grp));
		u32 cmp = val;

		if (byte_swap)
			cmp = ntohl((__force __be32)val);

		switch (cmp) {
		case XTR_NOT_READY:
			break;
		case XTR_ABORT:
			/* No accompanying data */
			abort_flag = true;
			eof_flag = true;
			break;
		case XTR_EOF_0:
		case XTR_EOF_1:
		case XTR_EOF_2:
		case XTR_EOF_3:
			/* This assumes STATUS_WORD_POS == 1, Status
			 * just after last data
			 */
			byte_cnt -= (4 - XTR_VALID_BYTES(val));
			eof_flag = true;
			break;
		case XTR_PRUNED:
			/* But get the last 4 bytes as well */
			eof_flag = true;
			pruned_flag = true;
			fallthrough;
		case XTR_ESCAPE:
			*rxbuf = spx5_rd(sparx5, QS_XTR_RD(grp));
			byte_cnt += 4;
			rxbuf++;
			break;
		default:
			*rxbuf = val;
			byte_cnt += 4;
			rxbuf++;
		}
	}

	if (abort_flag || pruned_flag || !eof_flag) {
		netdev_err(netdev, "Discarded frame: abort:%d pruned:%d eof:%d\n",
			   abort_flag, pruned_flag, eof_flag);
		kfree_skb(skb);
		return;
	}

#if defined(CONFIG_DEBUG_KERNEL) /* TODO: Remove before upstreaming */
	if (!netif_oper_up(netdev)) {
		netdev_err(netdev, "Discarded frame: Interface not up\n");
		kfree_skb(skb);
		return;
	}
#endif

	/* Everything we see on an interface that is in the HW bridge
	 * has already been forwarded
	 */
	if (test_bit(port->portno, sparx5->bridge_mask))
		skb->offload_fwd_mark = 1;

	/* Finish up skb */
	skb_put(skb, byte_cnt - ETH_FCS_LEN);
	eth_skb_pad(skb);
	skb->protocol = eth_type_trans(skb, netdev);
	netif_rx(skb);
	netdev->stats.rx_bytes += skb->len;
	netdev->stats.rx_packets++;
}

static int sparx5_inject(struct sparx5 *sparx5,
			 u32 *ifh,
			 struct sk_buff *skb)
{
	u32 val, w, count;
	int grp = INJ_QUEUE;
	u8 *buf;

	val = spx5_rd(sparx5, QS_INJ_STATUS);
	if (!(QS_INJ_STATUS_FIFO_RDY_GET(val) & BIT(grp))) {
		pr_err("Injection: Queue not ready: 0x%lx\n",
		       QS_INJ_STATUS_FIFO_RDY_GET(val));
		return -EBUSY;
	}

	if (QS_INJ_STATUS_WMARK_REACHED_GET(val) & BIT(grp)) {
		pr_err("Injection: Watermark reached: 0x%lx\n",
		       QS_INJ_STATUS_WMARK_REACHED_GET(val));
		return -EBUSY;
	}

	/* Indicate SOF */
	spx5_wr(QS_INJ_CTRL_SOF_SET(1) |
		QS_INJ_CTRL_GAP_SIZE_SET(1),
		sparx5, QS_INJ_CTRL(grp));

	// Write the IFH to the chip.
	for (w = 0; w < IFH_LEN; w++)
		spx5_wr(ifh[w], sparx5, QS_INJ_WR(grp));

	/* Write words, round up */
	count = ((skb->len + 3) / 4);
	buf = skb->data;
	for (w = 0; w < count; w++, buf += 4) {
		val = get_unaligned((const u32 *)buf);
		spx5_wr(val, sparx5, QS_INJ_WR(grp));
	}

	/* Add padding */
	while (w < (60 / 4)) {
		spx5_wr(0, sparx5, QS_INJ_WR(grp));
		w++;
	}

	/* Indicate EOF and valid bytes in last word */
	spx5_wr(QS_INJ_CTRL_GAP_SIZE_SET(1) |
		QS_INJ_CTRL_VLD_BYTES_SET(skb->len < 60 ? 0 : skb->len % 4) |
		QS_INJ_CTRL_EOF_SET(1),
		sparx5, QS_INJ_CTRL(grp));

	/* Add dummy CRC */
	spx5_wr(0, sparx5, QS_INJ_WR(grp));
	w++;

	return NETDEV_TX_OK;
}

int sparx5_port_xmit_impl(struct sk_buff *skb, struct net_device *dev)
{
	struct sparx5_port *port = netdev_priv(dev);
	struct sparx5 *sparx5 = port->sparx5;
	struct net_device_stats *stats = &dev->stats;
	int ret;

	ret = sparx5_inject(sparx5, port->ifh, skb);

	if (ret == NETDEV_TX_OK) {
		stats->tx_bytes += skb->len;
		stats->tx_packets++;
	} else {
		stats->tx_dropped++;
	}

	dev_kfree_skb_any(skb);

	return ret;
}

void sparx5_manual_injection_mode(struct sparx5 *sparx5)
{
	const int byte_swap = 1;
	int portno;

	/* Change mode to manual extraction and injection */
	spx5_wr(QS_XTR_GRP_CFG_MODE_SET(1) |
		QS_XTR_GRP_CFG_STATUS_WORD_POS_SET(1) |
		QS_XTR_GRP_CFG_BYTE_SWAP_SET(byte_swap),
		sparx5, QS_XTR_GRP_CFG(XTR_QUEUE));
	spx5_wr(QS_INJ_GRP_CFG_MODE_SET(1) |
		QS_INJ_GRP_CFG_BYTE_SWAP_SET(byte_swap),
		sparx5, QS_INJ_GRP_CFG(INJ_QUEUE));

	/* CPU ports capture setup */
	for (portno = SPX5_PORT_CPU_0; portno <= SPX5_PORT_CPU_1; portno++) {
		/* ASM CPU port: No preamble, IFH, enable padding */
		spx5_wr(ASM_PORT_CFG_PAD_ENA_SET(1) |
			ASM_PORT_CFG_NO_PREAMBLE_ENA_SET(1) |
			ASM_PORT_CFG_INJ_FORMAT_CFG_SET(1), /* 1 = IFH */
			sparx5, ASM_PORT_CFG(portno));
	}

	/* Reset WM cnt to unclog queued frames */
	for (portno = SPX5_PORT_CPU_0; portno <= SPX5_PORT_CPU_1; portno++)
		spx5_rmw(DSM_DEV_TX_STOP_WM_CFG_DEV_TX_CNT_CLR_SET(1),
			 DSM_DEV_TX_STOP_WM_CFG_DEV_TX_CNT_CLR,
			 sparx5,
			 DSM_DEV_TX_STOP_WM_CFG(portno));
}

irqreturn_t sparx5_xtr_handler(int irq, void *_sparx5)
{
	struct sparx5 *s5 = _sparx5;
	int poll = 64;

	/* Check data in queue */
	while (spx5_rd(s5, QS_XTR_DATA_PRESENT) & BIT(XTR_QUEUE) && poll-- > 0)
		sparx5_xtr_grp(s5, XTR_QUEUE, false);

	return IRQ_HANDLED;
}
