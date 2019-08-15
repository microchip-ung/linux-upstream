// SPDX-License-Identifier: (GPL-2.0 OR MIT)
//
// Microsemi SoC FDMA IFH driver
//
// Copyright (c) 2019 Microsemi

#include <linux/dmaengine.h>
#include <linux/etherdevice.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/rtnetlink.h>

#define DEVNAME             "vtss.ifh"
#define ETH_VLAN_TAGSZ     4    /* Size of a 802.1Q VLAN tag */
#define RX_MTU_DEFAULT     (ETH_FRAME_LEN + ETH_FCS_LEN + (2 * ETH_VLAN_TAGSZ))
#define RX_MTU_MIN         64
#define RX_MTU_MAX         16384
#define IF_BUFSIZE_JUMBO   10400
#define SGL_MAX            15

#define FDMA_REQUEST_THRESHOLD     (64*1024)
#define FDMA_REQUEST_MAX           100
#define FDMA_XTR_BUFFER_COUNT      SGL_MAX
#define FDMA_XTR_BUFFER_SIZE       2048

#define VITESSE_ENCAP_SIZE     16
#define VITESSE_ETHTYPE_OFFSET 12
#define VITESSE_IFH_ID_OFFSET  15
#define VITESSE_ETHTYPE_MSB    0x88
#define VITESSE_ETHTYPE_LSB    0x80

/* ASM:CFG:PORT_CFG[67] */
#define ASM_CFG_PORT_R_OFF(ridx)                   (0x0060841c + (ridx*4))

/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_R_OFF(ridx)          (0x01030000 + (ridx*4))
/* DEVCPU_QS:XTR:XTR_RD[2] */
#define DEVCPU_QS_XTR_RD_R_OFF(ridx)               (0x01030008 + (ridx*4))
/* DEVCPU_QS:XTR:XTR_FRM_PRUNING[2] */
#define DEVCPU_QS_XTR_FRM_PRUNING_R_OFF(ridx)      (0x01030010 + (ridx*4))
/* DEVCPU_QS:XTR:XTR_FLUSH */
#define DEVCPU_QS_XTR_FLUSH_OFF                    0x01030018
/* DEVCPU_QS:XTR:XTR_DATA_PRESENT */
#define DEVCPU_QS_XTR_DATA_PRESENT_OFF             0x0103001c
/* DEVCPU_QS:XTR:XTR_CFG */
#define DEVCPU_QS_XTR_CFG_OFF                      0x01030020
/* DEVCPU_QS:INJ:INJ_GRP_CFG[2] */
#define DEVCPU_QS_INJ_GRP_CFG_R_OFF(ridx)          (0x01030024 + (ridx*4))
/* DEVCPU_QS:INJ:INJ_WR[2] */
#define DEVCPU_QS_INJ_WR_R_OFF(ridx)               (0x0103002c + (ridx*4))
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_R_OFF(ridx)             (0x01030034 + (ridx*4))
/* DEVCPU_QS:INJ:INJ_STATUS */
#define DEVCPU_QS_INJ_STATUS_OFF                   0x0103003c
/* DEVCPU_QS:INJ:INJ_ERR[2] */
#define DEVCPU_QS_INJ_ERR_R_OFF(ridx)              (0x01030040 + (ridx*4))
/* DEVCPU_QS:INJ:VTSS_DBG */
#define DEVCPU_QS_INJ_VTSS_DBG_OFF                 0x01030048

/* DSM:CFG:DEV_TX_STOP_WM_CFG[67] */
#define DSM_CFG_DEV_TX_STOP_WM_R_OFF(ridx)         (0x00504564 + (ridx*4))

/* QSYS:SYSTEM:FRM_AGING */
#define QSYS_SYSTEM_FRM_AGING_OFF                  0x010a0108

/* REW:COMMON:IFH_CTRL_CPUVD */
#define REW_COMMON_IFH_CTRL_CPUVD_OFF              0x0165e9d4


/* ASM:CFG:PORT_CFG[67] */
#define ASM_CFG_PORT_NO_PREAMBLE_ENA             BIT(9)
#define ASM_CFG_PORT_FRM_AGING_DIS               BIT(7)
#define ASM_CFG_PORT_INJ_FORMAT(x)               (((x) << 2) & GENMASK(3, 2))
#define ASM_CFG_PORT_INJ_FORMAT_M                GENMASK(3, 2)
#define ASM_CFG_PORT_INJ_FORMAT_X(x)             (((x) & GENMASK(3, 2)) >> 2)
#define ASM_CFG_PORT_VSTAX2_AWR_ENA              BIT(1)

/* DSM:CFG:DEV_TX_STOP_WM_CFG[67] */
#define DSM_CFG_DEV_TX_STOP_WM(x)                (((x) << 1) & GENMASK(7, 1))
#define DSM_CFG_DEV_TX_STOP_WM_M                 GENMASK(7, 1)
#define DSM_CFG_DEV_TX_STOP_WM_X(x)              (((x) & GENMASK(7, 1)) >> 1)

/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_MODE(x)            (((x) << 2) & GENMASK(3, 2))
#define DEVCPU_QS_XTR_GRP_CFG_MODE_M             GENMASK(3, 2)
#define DEVCPU_QS_XTR_GRP_CFG_MODE_X(x)          (((x) & GENMASK(3, 2)) >> 2)
/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_STATUS_WORD_POS    BIT(1)
/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_BYTE_SWAP          BIT(0)
/* DEVCPU_QS:XTR:XTR_RD[2] */
#define DEVCPU_QS_XTR_RD_DATA(x)                 ((x) & GENMASK(31, 0))
/* DEVCPU_QS:XTR:XTR_FRM_PRUNING[2] */
#define DEVCPU_QS_XTR_FRM_PRUNING_PRUNE_SIZE(x)  ((x) & GENMASK(7, 0))
/* DEVCPU_QS:XTR:XTR_FLUSH */
#define DEVCPU_QS_XTR_FLUSH(x)                   ((x) & GENMASK(1, 0))
/* DEVCPU_QS:XTR:XTR_DATA_PRESENT */
#define DEVCPU_QS_XTR_DATA_PRESENT(x)            ((x) & GENMASK(1, 0))
/* DEVCPU_QS:XTR:XTR_CFG */
#define DEVCPU_QS_XTR_CFG_DP_WM(x)               (((x) << 7) & GENMASK(11, 7))
#define DEVCPU_QS_XTR_CFG_DP_WM_M                GENMASK(11, 7)
#define DEVCPU_QS_XTR_CFG_DP_WM_X(x)             (((x) & GENMASK(11, 7)) >> 7)
/* DEVCPU_QS:XTR:XTR_CFG */
#define DEVCPU_QS_XTR_CFG_SCH_WM(x)              (((x) << 2) & GENMASK(6, 2))
#define DEVCPU_QS_XTR_CFG_SCH_WM_M               GENMASK(6, 2)
#define DEVCPU_QS_XTR_CFG_SCH_WM_X(x)            (((x) & GENMASK(6, 2)) >> 2)
/* DEVCPU_QS:XTR:XTR_CFG */
#define DEVCPU_QS_XTR_CFG_OFLW_ERR_STICKY(x)     ((x) & GENMASK(1, 0))
#define DEVCPU_QS_XTR_CFG_OFLW_ERR_STICKY_M      GENMASK(1, 0)
/* DEVCPU_QS:INJ:INJ_GRP_CFG[2] */
#define DEVCPU_QS_INJ_GRP_CFG_MODE(x)            (((x) << 2) & GENMASK(3, 2))
#define DEVCPU_QS_INJ_GRP_CFG_MODE_M             GENMASK(3, 2)
#define DEVCPU_QS_INJ_GRP_CFG_MODE_X(x)          (((x) & GENMASK(3, 2)) >> 2)
/* DEVCPU_QS:INJ:INJ_GRP_CFG[2] */
#define DEVCPU_QS_INJ_GRP_CFG_BYTE_SWAP          BIT(0)
/* DEVCPU_QS:INJ:INJ_WR[2] */
#define DEVCPU_QS_INJ_WR_DATA(x)                 ((x) & GENMASK(31, 0))
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_GAP_SIZE(x)           (((x) << 21) & GENMASK(24, 21))
#define DEVCPU_QS_INJ_CTRL_GAP_SIZE_M            GENMASK(24, 21)
#define DEVCPU_QS_INJ_CTRL_GAP_SIZE_X(x)         (((x) & GENMASK(24, 21)) >> 21)
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_ABORT                 BIT(20)
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_EOF                   BIT(19)
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_SOF                   BIT(18)
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_VLD_BYTES(x)          (((x) << 16) & GENMASK(17, 16))
#define DEVCPU_QS_INJ_CTRL_VLD_BYTES_M           GENMASK(17, 16)
#define DEVCPU_QS_INJ_CTRL_VLD_BYTES_X(x)        (((x) & GENMASK(17, 16)) >> 16)
/* DEVCPU_QS:INJ:INJ_STATUS */
#define DEVCPU_QS_INJ_STATUS_WMARK_REACHED_M     GENMASK(5, 4)
#define DEVCPU_QS_INJ_STATUS_WMARK_REACHED_X(x)  (((x) & GENMASK(5, 4)) >> 4)
/* DEVCPU_QS:INJ:INJ_STATUS */
#define DEVCPU_QS_INJ_STATUS_FIFO_RDY_M          GENMASK(3, 2)
#define DEVCPU_QS_INJ_STATUS_FIFO_RDY_X(x)       (((x) & GENMASK(3, 2)) >> 2)
/* DEVCPU_QS:INJ:INJ_STATUS */
#define DEVCPU_QS_INJ_STATUS_IN_PROGRESS(x)      ((x) & GENMASK(1, 0))
#define DEVCPU_QS_INJ_STATUS_IN_PROGRESS_M       GENMASK(1, 0)

#define XTR_EOF_0          0x00000080U
#define XTR_EOF_1          0x01000080U
#define XTR_EOF_2          0x02000080U
#define XTR_EOF_3          0x03000080U
#define XTR_PRUNED         0x04000080U
#define XTR_ABORT          0x05000080U
#define XTR_ESCAPE         0x06000080U
#define XTR_NOT_READY      0x07000080U
#define XTR_VALID_BYTES(x) (4 - (((x) >> 24) & 3))

#define use_fdma(p)                (p->rxdma && p->txdma)

/* Logging options */

/* Testing options */
#define DEBUG_RX
#define DEBUG_VCORE_ACCESS

static int mscc_ifh_module_retry = 0;

struct fdma_config {
	u8 first_cpu_port;
	u8 ifh_id;
	u16 ifh_len;
	u16 ifh_encap_len;
};

struct mscc_ifh_priv;

struct mscc_ifh_request {
	struct list_head node;
	struct mscc_ifh_priv *priv;
	enum dma_data_direction direction;
	dma_cookie_t cookie;
	struct sk_buff *skb;
	unsigned int size;
	unsigned int blocks;
	struct scatterlist sgl[SGL_MAX];
	void *buffer[SGL_MAX];
};

struct request_iterator {
	int idx;
	struct mscc_ifh_request *req;
};

struct mscc_ifh_priv {
	struct device *dev;
	struct net_device *netdev;
	const struct fdma_config *config;
	struct dma_chan *rxdma;
	struct dma_chan *txdma;
	struct list_head free_reqs;
	struct list_head tx_reqs;
	struct list_head rx_reqs;
	int inj_port;
	void __iomem *io_addr;
	struct tasklet_struct tasklet;
	int available_rx_blocks;
	struct request_iterator curiter;
	int allocations;
	long unsigned tx_packets;
	long unsigned tx_bytes;
	long unsigned rx_packets;
	long unsigned rx_bytes;
	spinlock_t lock;
};

#if defined(DEBUG_RX) || defined(DEBUG_VCORE_ACCESS)
static void mscc_ifh_dump_byte_array(const char *name, const u8 *buf, size_t len)
{
	char prefix[64];
	if (!buf) {
		return;
	}
	snprintf(prefix, sizeof(prefix), "%.50s[%zu]: ", name, len);
	print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, buf, len,
		       false);
}
#endif

/* Forward Declarations */
static void mscc_ifh_provide_blocks(struct mscc_ifh_priv *priv, 
				      int blocks, int size);


static void mscc_ifh_destruct_skb(struct sk_buff *skb)
{
	struct mscc_ifh_priv *priv;

	pr_debug("%s:%d %s: skb head (not freed here): 0x%px, dev: 0x%px\n", 
		__FILE__, __LINE__, __func__, skb->head, skb->dev);
	priv = netdev_priv(skb->dev);
	priv->allocations--;
	pr_debug("%s:%d %s: allocations: %d\n",
		__FILE__, __LINE__, __func__, priv->allocations);
}

static void mscc_ifh_writel(struct mscc_ifh_priv *priv, u32 reg, u32 data)
{
	void __iomem *addr = priv->io_addr + reg;
#ifdef DEBUG_VCORE_ACCESS
	pr_debug("%s:%d %s: addr 0x%px, data 0x%08x\n", 
		__FILE__, __LINE__, __func__,
		addr,
		data);
#endif
	writel(data, addr);
}

static u32 mscc_ifh_readl(struct mscc_ifh_priv *priv, u32 reg)
{
	void __iomem *addr = priv->io_addr + reg;
	u32 data = readl(addr);
#ifdef DEBUG_VCORE_ACCESS
	pr_debug("%s:%d %s: addr 0x%px, data 0x%08x\n", 
		__FILE__, __LINE__, __func__, 
		addr,
		data);
#endif
	return data;
}

static int mscc_ifh_xtr_word(struct mscc_ifh_priv *priv, bool first_word,
			     u32 *rval, u32 *bytes_valid)
{
	u32 val;

	*bytes_valid = 0;
	val = mscc_ifh_readl(priv, DEVCPU_QS_XTR_RD_R_OFF(priv->inj_port));
	if (val == XTR_NOT_READY) {
		/*
		 * XTR_NOT_READY means two different things depending on whether 
		 * this is the first word read of a frame or after at least one 
		 * word has been read. When the first word, the group is empty, 
		 * and we return an error. Otherwise we have to wait for the 
		 * FIFO to have received some more data. 
		 */
		if (first_word) {
			return 2; /* Error */
		}
		do {
			val = mscc_ifh_readl(priv, 
				DEVCPU_QS_XTR_RD_R_OFF(priv->inj_port));
		} while (val == XTR_NOT_READY);
	}

	switch (val) {
	case XTR_ABORT:
		return 2; /* Error */
	case XTR_EOF_0:
	case XTR_EOF_1:
	case XTR_EOF_2:
	case XTR_EOF_3:
	case XTR_PRUNED:
		*bytes_valid = XTR_VALID_BYTES(val);
		val = mscc_ifh_readl(priv, DEVCPU_QS_XTR_RD_R_OFF(priv->inj_port));
		if (val == XTR_ESCAPE) {
			*rval = mscc_ifh_readl(priv, 
					DEVCPU_QS_XTR_RD_R_OFF(priv->inj_port));
		} else {
			*rval = val;
		}
		return 1; /* EOF */
	case XTR_ESCAPE:
		*rval = mscc_ifh_readl(priv, 
				DEVCPU_QS_XTR_RD_R_OFF(priv->inj_port));
		*bytes_valid = 4;
		return 0;
	default:
		*rval = val;
		*bytes_valid = 4;
		return 0;
	}
}

static void mscc_ifh_xtr_tasklet(unsigned long arg)
{
	struct mscc_ifh_priv *priv = (struct mscc_ifh_priv *)arg;
	struct sk_buff *skb;
	u32 *wbuf;
	int buf_len = 4*1024;
	int header_word_size;
	bool done = 0;
	u32 val;
	u32 bytes_valid;
	u32 size = 0;
	int idx;
	int mask = BIT(priv->inj_port);

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	skb = netdev_alloc_skb(priv->netdev, buf_len);
	if (unlikely(!skb)) {
		netdev_err(priv->netdev, "Unable to allocate sk_buff\n");
		goto error;
	}

	wbuf = (u32 *)skb->data;

	size = priv->config->ifh_encap_len + priv->config->ifh_len;
	header_word_size = size / 4;

	for (idx = 0; idx < header_word_size; ++idx) {
		if (mscc_ifh_xtr_word(priv, 1, &val, &bytes_valid) != 0) {
			goto error;
		}
		*wbuf++ = val;
	}
#ifdef DEBUG
	mscc_ifh_dump_byte_array("RxHeader", skb->data, header_word_size * 4);
#endif
	/* Read the rest of the frame */
	while (!done && buf_len >= 4) {
		u32 result = mscc_ifh_xtr_word(priv, 0, &val, &bytes_valid);
		if (result == 2) {
			pr_debug("%s:%d %s: abort\n", __FILE__, __LINE__, __func__);
			goto error;
		}
		*wbuf++ = val;
		buf_len -= bytes_valid;
		size += bytes_valid;
		done = result == 1;
	}
	if (!done) {
		goto error;
	}
	/* Skip to the IFH encap IFH type */
	skb_put(skb, size);
#ifdef DEBUG
	mscc_ifh_dump_byte_array("RxSkb", skb->data, skb->len);
#endif
	skb->protocol = eth_type_trans(skb, priv->netdev);
	netif_rx(skb);
	priv->netdev->stats.rx_bytes += size;
	priv->netdev->stats.rx_packets++;
	return;

error:
	pr_debug("%s:%d %s: start error reading\n", __FILE__, __LINE__, __func__);
	while (mscc_ifh_readl(priv, DEVCPU_QS_XTR_DATA_PRESENT_OFF) & mask) {
		mscc_ifh_readl(priv, DEVCPU_QS_XTR_RD_R_OFF(priv->inj_port));
	}
	mscc_ifh_writel(priv, DEVCPU_QS_XTR_FLUSH_OFF, mask);
	mscc_ifh_writel(priv, DEVCPU_QS_XTR_FLUSH_OFF, 0);
	pr_debug("%s:%d %s: end error reading\n", __FILE__, __LINE__, __func__);
}


static irqreturn_t mscc_ifh_xtr_irq_handler(int irq, void *arg)
{
	struct mscc_ifh_priv *priv = arg;
	int mask = BIT(priv->inj_port);

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	if (!(mscc_ifh_readl(priv, DEVCPU_QS_XTR_DATA_PRESENT_OFF) & mask)) {
		return IRQ_NONE;
	}

	tasklet_schedule(&priv->tasklet);

	return IRQ_HANDLED;
}

static int mscc_ifh_transmit_register_based(struct mscc_ifh_priv *priv, 
					  struct sk_buff *skb)
{
	u32 status;
	u32 mask = BIT(priv->inj_port);
	int idx;
	int count, last;
	u32 *buffer;

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	/* Skip past the encapsulation header */
	skb_pull_inline(skb, VITESSE_ENCAP_SIZE);
#ifdef DEBUG
	mscc_ifh_dump_byte_array("TxSkb", skb->data, skb->len);
#endif
	status = mscc_ifh_readl(priv, DEVCPU_QS_INJ_STATUS_OFF);
	if (!(DEVCPU_QS_INJ_STATUS_FIFO_RDY_X(status) & mask)) {
		pr_debug("%s:%d %s: FIFO not ready\n", 
			 __FILE__, __LINE__, __func__);
		return -1;
	}
	if (DEVCPU_QS_INJ_STATUS_WMARK_REACHED_X(status) & mask) {
		pr_debug("%s:%d %s: Watermark reached\n", 
			 __FILE__, __LINE__, __func__);
		return -1;
	}
	/* Indicate SOF */
	mscc_ifh_writel(priv, DEVCPU_QS_INJ_CTRL_R_OFF(priv->inj_port), \
			DEVCPU_QS_INJ_CTRL_GAP_SIZE(1) | DEVCPU_QS_INJ_CTRL_SOF);

	/* Write frame words and round up to whole words (32 bit) */
	count = ((skb->len + 3) / 4);
	last = skb->len % 4;
	buffer = (u32 *)skb->data;
	for (idx = 0; idx < count; idx++, skb->data += 4) {
		pr_debug("%s:%d %s: Data: %08x\n", 
			 __FILE__, __LINE__, __func__, *buffer);
		mscc_ifh_writel(priv, DEVCPU_QS_INJ_WR_R_OFF(priv->inj_port),
				*buffer++);
	}

	/* Add padding to reach the minimum packet size for the VCore */
	while (idx < (60 / 4)) {
		mscc_ifh_writel(priv, DEVCPU_QS_INJ_WR_R_OFF(priv->inj_port), 0);
		idx++;
	}
	/* Indicate EOF and valid bytes in last word */
	mscc_ifh_writel(priv, DEVCPU_QS_INJ_CTRL_R_OFF(priv->inj_port), \
			DEVCPU_QS_INJ_CTRL_GAP_SIZE(1) |
			DEVCPU_QS_INJ_CTRL_VLD_BYTES(skb->len < 60 ? 0 : last) |
			DEVCPU_QS_INJ_CTRL_EOF);
	/* Add dummy CRC */
	mscc_ifh_writel(priv, DEVCPU_QS_INJ_WR_R_OFF(priv->inj_port), 0);
	idx++;
	pr_debug("%s:%d %s: written: %d bytes\n", 
		 __FILE__, __LINE__, __func__, idx * 4);

	return 0;
}

static void mscc_ifh_register_based_injection(struct mscc_ifh_priv *priv)
{
	unsigned int chipport = priv->inj_port + priv->config->first_cpu_port;
	u32 value, mask;

	/* Allow manual injection via DEVCPU_QS registers, and byte swap
	 * these registes endianness.
	 */
	mscc_ifh_writel(priv, DEVCPU_QS_INJ_GRP_CFG_R_OFF(priv->inj_port),
			DEVCPU_QS_INJ_GRP_CFG_BYTE_SWAP | 
			DEVCPU_QS_INJ_GRP_CFG_MODE(1));
	mscc_ifh_writel(priv, DEVCPU_QS_XTR_GRP_CFG_R_OFF(priv->inj_port),
			DEVCPU_QS_XTR_GRP_CFG_BYTE_SWAP | 
			DEVCPU_QS_XTR_GRP_CFG_MODE(1));

	/* INJ CPU Port Format: IFH without prefix, no preamble, no VSTAX2 aware */
	value = mscc_ifh_readl(priv, ASM_CFG_PORT_R_OFF(chipport));
	value &= ~ASM_CFG_PORT_VSTAX2_AWR_ENA;
	value |= ASM_CFG_PORT_INJ_FORMAT(1);
#ifdef DEBUG_VCORE_ACCESS
	value |= ASM_CFG_PORT_FRM_AGING_DIS;
#endif
	value |= ASM_CFG_PORT_NO_PREAMBLE_ENA;
	mscc_ifh_writel(priv, ASM_CFG_PORT_R_OFF(chipport), value);

	/* XTR CPU Port Format: IFH with prefix */
	value = mscc_ifh_readl(priv, REW_COMMON_IFH_CTRL_CPUVD_OFF);
	mask = ~(0x3 << (priv->inj_port << 1));
	value &= mask;
	value |= 2 << (priv->inj_port << 1);
	mscc_ifh_writel(priv, REW_COMMON_IFH_CTRL_CPUVD_OFF, value);

	/* XTR: Set Disassembler Stop Watermark level */
	value = mscc_ifh_readl(priv, DSM_CFG_DEV_TX_STOP_WM_R_OFF(chipport));
	value &= ~DSM_CFG_DEV_TX_STOP_WM_M;
	value |= DSM_CFG_DEV_TX_STOP_WM(10);  /* Watermark at 10 packets */
	mscc_ifh_writel(priv, DSM_CFG_DEV_TX_STOP_WM_R_OFF(chipport), value);
}

static struct mscc_ifh_request *mscc_ifh_prepare_tx_request(
	struct mscc_ifh_priv *priv,
	enum dma_data_direction dir,
	struct sk_buff *skb)
{
	struct mscc_ifh_request *req;
	struct scatterlist *sg;
	int idx;
	void *data;
	dma_addr_t addr;
	int blocks;
	unsigned int size;

	blocks = skb_shinfo(skb)->nr_frags + 1;
	pr_debug("%s:%d %s: skb: frags: %d, size: %u, headsize: %u\n", 
		__FILE__, __LINE__, __func__,
		skb_shinfo(skb)->nr_frags,
		skb->len,
		skb_headlen(skb));
	if (blocks > SGL_MAX) {
		pr_err("%s:%d %s: too many blocks\n", 
		       __FILE__, __LINE__, __func__);
		return NULL;
	}
	req = list_first_entry_or_null(&priv->free_reqs, struct mscc_ifh_request, 
				       node);
	if (!req) {
		return NULL;
	}
	memset(req->sgl, 0, sizeof(req->sgl));
	memset(req->buffer, 0, sizeof(req->buffer));
	req->priv = priv;
	req->direction = dir;
	req->cookie = 0;
	req->skb = skb;
	req->size = skb->len;
	req->blocks = blocks;
	sg_init_table(req->sgl, blocks);
	sg = req->sgl;

	/* Check encapsulation header (MAC + VLAN Tag) */
	if (!(skb->data[VITESSE_ETHTYPE_OFFSET] == VITESSE_ETHTYPE_MSB &&
		skb->data[VITESSE_ETHTYPE_OFFSET+1] == VITESSE_ETHTYPE_LSB &&
		skb->data[VITESSE_IFH_ID_OFFSET] == priv->config->ifh_id)) {
		pr_debug("%s:%d %s: encapsulation check failed: %d, %d, %d\n",
		       __FILE__, __LINE__, __func__,
		       skb->data[VITESSE_ETHTYPE_OFFSET] == VITESSE_ETHTYPE_MSB,
		       skb->data[VITESSE_ETHTYPE_OFFSET+1] == VITESSE_ETHTYPE_LSB,
		       skb->data[VITESSE_IFH_ID_OFFSET] == priv->config->ifh_id);
		/* Count as dropped */
		priv->netdev->stats.tx_dropped++;
		kfree_skb(skb);
		return NULL;
	}
	/* Skip past the encapsulation header */
	skb_pull_inline(skb, VITESSE_ENCAP_SIZE);
	/* Add room for the FCS  needed_tailroom = ETH_FCS_LEN? */
	skb_put(skb, ETH_FCS_LEN);

	data = skb->data;
	size = skb_headlen(skb); /* Includes the IFH and FCS */
	addr = dma_map_single(priv->dev, data, size, dir);
	if (unlikely(dma_mapping_error(priv->dev, addr))) {
		goto out;
	}
	sg_dma_address(sg) = addr;
	sg_dma_len(sg) = size;
	sg = sg_next(sg);
	/* Add SKB fragments if available */
	for (idx = 0; idx < skb_shinfo(skb)->nr_frags; idx++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[idx];

		if (!skb_frag_size(frag)) {
			sg_dma_address(sg) = 0;
			sg_dma_len(sg) = 0;
			continue;
		}
		addr = skb_frag_dma_map(priv->dev, frag, 0,
					skb_frag_size(frag), dir);
		if (unlikely(dma_mapping_error(priv->dev, addr))) {
			goto out;
		}
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = skb_frag_size(frag);
		sg = sg_next(sg);
	}
	list_move_tail(&req->node, &priv->tx_reqs);
	return req;
out:
	pr_err("%s:%d %s: mapping error\n",
	       __FILE__, __LINE__, __func__);
	return NULL;
}

static struct mscc_ifh_request *mscc_ifh_prepare_rx_request(
	struct mscc_ifh_priv *priv,
	enum dma_data_direction dir,
	int blocks, int size)
{
	struct mscc_ifh_request *req;
	struct scatterlist *sg;
	int idx;
	void *data;
	dma_addr_t addr;

	pr_debug("%s:%d %s: blocks: %u\n", __FILE__, __LINE__, __func__, 
		blocks);
	if (blocks > SGL_MAX) {
		pr_err("%s:%d %s: too many blocks\n", 
		       __FILE__, __LINE__, __func__);
		return 0;
	}
	req = list_first_entry_or_null(&priv->free_reqs, struct mscc_ifh_request, 
				       node);
	if (!req) {
		return NULL;
	}
	memset(req->sgl, 0, sizeof(req->sgl));
	memset(req->buffer, 0, sizeof(req->buffer));
	req->priv = priv;
	req->direction = dir;
	req->cookie = 0;
	req->skb = 0;
	req->size = size;
	size += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	req->blocks = blocks;
	sg_init_table(req->sgl, blocks);
	for_each_sg(req->sgl, sg, blocks, idx) {
		data = kzalloc(size, GFP_KERNEL);
		req->buffer[idx] = data;
		priv->allocations++;
		addr = dma_map_single(priv->dev, data, size, dir);
		if (unlikely(dma_mapping_error(priv->dev, addr))) {
			goto out;
		}
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = req->size;
		pr_debug("%s:%d %s: data: 0x%px, size: %u, shinfo: %lu, phys: 0x%llx\n",
			__FILE__, __LINE__, __func__, 
			data,
			req->size,
			SKB_DATA_ALIGN(sizeof(struct skb_shared_info)),
			addr);
	}
	list_move_tail(&req->node, &priv->rx_reqs);
	return req;
out:
	pr_err("%s:%d %s: mapping error\n",
	       __FILE__, __LINE__, __func__);
	return NULL;
}

static void mscc_ifh_close_request(struct mscc_ifh_priv *priv,
				   struct mscc_ifh_request *req,
				   bool unmap, bool free)
{
	pr_debug("%s:%d %s: [C%u]\n",
		__FILE__, __LINE__, __func__, req->cookie);
	if (unmap) {
		struct scatterlist *sg;
		int idx;

		for_each_sg(req->sgl, sg, req->blocks, idx) {
			dma_unmap_single(priv->dev,
					 sg_dma_address(sg),
					 sg_dma_len(sg),
					 req->direction);
			if (free) {
				pr_debug("%s:%d %s: free: 0x%px [I%u]\n",
					 __FILE__, __LINE__, __func__,
					 req->buffer[idx], idx);
				kfree(req->buffer[idx]);
			}
		}
	}
	list_move_tail(&req->node, &priv->free_reqs);
}

static void mscc_ifh_init_iterator(struct request_iterator *iter, 
	int idx,
	struct mscc_ifh_request *req)
{
	iter->idx = idx;
	iter->req = req;
	if (idx == iter->req->blocks) {
		iter->idx = 0;
		iter->req = list_next_entry(iter->req, node);
	}
	pr_debug("%s:%d %s: [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx);
}

static void mscc_ifh_copy_iterator(struct request_iterator *iter,
	struct request_iterator *other)

{
	iter->idx = other->idx;
	iter->req = other->req;
	pr_debug("%s:%d %s: [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx);
}

static struct mscc_ifh_request *next_block(struct request_iterator *iter)
{
	struct mscc_ifh_request *req = NULL;

	iter->idx++;
	if (iter->idx == iter->req->blocks) {
		req = iter->req;
		iter->idx = 0;
		iter->req = list_next_entry(iter->req, node);
	}
	pr_debug("%s:%d %s: [C%u,I%u], req: %u\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx, 
		req != NULL);
	return req;
}

static bool mscc_ifh_end_of_packet(struct request_iterator *iter,
	struct request_iterator *max)
{
	if (iter->req != max->req) {
		if (iter->idx + 1 == iter->req->blocks) {
			struct mscc_ifh_request *req = 
				list_next_entry(iter->req, node);

			pr_debug("%s:%d %s: %u\n", __FILE__, __LINE__, __func__,
				req == max->req && max->idx == 0);
			return req == max->req && max->idx == 0;
		}
		pr_debug("%s:%d %s: 0\n", __FILE__, __LINE__, __func__);
		return 0;
	}
	pr_debug("%s:%d %s: %u\n", __FILE__, __LINE__, __func__,
		iter->idx + 1 == max->idx);
	return iter->idx + 1 == max->idx;
}

static bool mscc_ifh_reached(struct request_iterator *iter,
	struct request_iterator *max)
{
	pr_debug("%s:%d %s: %u\n", __FILE__, __LINE__, __func__,
		iter->req == max->req && iter->idx == max->idx);
	return iter->req == max->req && iter->idx == max->idx;
}

static void *mscc_ifh_get_block_data(struct mscc_ifh_priv *priv, 
				     struct request_iterator *iter)
{
	struct scatterlist *sg = &iter->req->sgl[iter->idx];

	dma_unmap_single(priv->dev,
			 sg_dma_address(sg),
			 sg_dma_len(sg),
			 iter->req->direction);
	pr_debug("%s:%d %s: [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx);
	return iter->req->buffer[iter->idx];
}

static struct sk_buff *mscc_ifh_create_receive_skb(
	struct mscc_ifh_priv *priv,
	struct request_iterator *iter,
	struct request_iterator *max,
	int *blks,
	u32 size)
{
	struct sk_buff *skb = 0;
	u32 *packet;
	void *data;
	unsigned int blocks;
	unsigned int block_bytes;
	struct mscc_ifh_request *done_req;

	pr_debug("%s:%d %s: from: [C%u,I%u] to: [C%u,I%u]\n", 
		 __FILE__, __LINE__, __func__,
		 iter->req->cookie,
		 iter->idx,
		 max->req->cookie,
		 max->idx);

	packet = data = mscc_ifh_get_block_data(priv, iter);
	/* Get the packet size (includes IFH and FCS) */
	blocks = DIV_ROUND_UP(size, iter->req->size);
	block_bytes = blocks * iter->req->size;
	pr_debug("%s:%d %s: data: 0x%px, bytes: %u, size: %u, blocks: %u, "
		 "block_bytes: %u\n",
		 __FILE__, __LINE__, __func__, 
		 data,
		 size,
		 iter->req->size,
		 blocks,
		 block_bytes);
	if (mscc_ifh_end_of_packet(iter, max)) {
		skb = build_skb(data, 0);
		if (!skb) {
			pr_err("%s:%d %s: no skb: %u bytes\n", 
			       __FILE__, __LINE__, __func__, iter->req->size);
#ifdef DEBUG
			mscc_ifh_dump_byte_array("RxData", data, 
				min(block_bytes, iter->req->size));
#endif
			goto out;
		}

#ifdef DEBUG_RX
		mscc_ifh_dump_byte_array("IFH RxData", data, 
			 min(size, iter->req->size));
#endif
		skb->dev = priv->netdev;
		skb->destructor = mscc_ifh_destruct_skb;
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n",
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		skb_put(skb, size);
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n", 
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		done_req = next_block(iter);
		if (done_req) {
			pr_debug("%s:%d %s: done: [C:%u]\n", 
				 __FILE__, __LINE__, __func__, done_req->cookie);
			list_move_tail(&done_req->node, &priv->free_reqs);
		}
	} else {
		void *ptr;

		skb = netdev_alloc_skb(priv->netdev, block_bytes);
		if (!skb) {
			pr_err("%s:%d %s: no skb: %u bytes\n", 
			       __FILE__, __LINE__, __func__, block_bytes);
			goto out_free;
		}

		ptr = skb->data;
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n",
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		skb_put(skb, size);
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n", 
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		while (!mscc_ifh_reached(iter, max)) {
			data = mscc_ifh_get_block_data(priv, iter);
#ifdef DEBUG
			mscc_ifh_dump_byte_array("RxData", data, iter->req->size);
#endif
			pr_debug("%s:%d %s: copy: len: %d, data: 0x%px\n", 
				 __FILE__, __LINE__, __func__, 
				 iter->req->size, data);
			memcpy(ptr, data, iter->req->size);
			kfree(data);
			priv->allocations--;
			done_req = next_block(iter);
			if (done_req) {
				pr_debug("%s:%d %s: done: [C:%u]\n", 
					 __FILE__, __LINE__, __func__, 
					 done_req->cookie);
				list_move_tail(&done_req->node, &priv->free_reqs);
			}
			ptr += iter->req->size;
		}
	}
	*blks = blocks;
	skb->protocol = eth_type_trans(skb, priv->netdev);
	pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n", 
		 __FILE__, __LINE__, __func__, skb->len, skb->data);
	return skb;

out_free:
	while (!mscc_ifh_reached(iter, max)) {
#ifdef DEBUG
		mscc_ifh_dump_byte_array("RxData", data, 
			min(block_bytes, iter->req->size));
#endif
		pr_debug("%s:%d %s: free: 0x%px\n",
			 __FILE__, __LINE__, __func__, data);
		kfree(data);
		priv->allocations--;
		done_req = next_block(iter);
		if (done_req) {
			pr_debug("%s:%d %s: done: [C:%u]\n", 
				 __FILE__, __LINE__, __func__, done_req->cookie);
			list_move_tail(&done_req->node, &priv->free_reqs);
		}
		data = mscc_ifh_get_block_data(priv, iter);
	}
out:
	*blks = 0;
	return 0;
}

static void mscc_ifh_receive_blocks_cb(void *data, 
	const struct dmaengine_result *result)
{
	struct mscc_ifh_request *req = data;
	struct mscc_ifh_priv *priv;
	int used_blocks;
	enum dma_status status;
	struct dma_tx_state state;
	struct sk_buff *skb;
	struct request_iterator next;
	int next_sof;

	pr_debug("%s:%d %s: result: %u, residue: %u\n", 
		__FILE__, __LINE__, __func__, 
		result->result, result->residue);
	if (!req) {
		pr_err("%s:%d %s: no request\n",
			__FILE__, __LINE__, __func__);
		return;
	}
	priv = req->priv;
	status = dmaengine_tx_status(priv->rxdma, req->cookie, &state);
	next_sof = req->blocks - (state.residue / req->size);
	mscc_ifh_init_iterator(&next, next_sof, req);
	pr_debug("%s:%d %s: status: %u, state: (L%u, U%u), size: %u, [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, 
		status,
		state.last, state.used, state.residue,
		req->cookie,
		next_sof);

	if (result->result == DMA_TRANS_NOERROR) {
		skb = mscc_ifh_create_receive_skb(priv, &priv->curiter, &next, 
			&used_blocks, result->residue);
		if (!skb) {
			pr_err("%s:%d %s: could not create skb: [C%u,I%u]\n", 
				__FILE__, __LINE__, __func__, 
				priv->curiter.req->cookie,
				priv->curiter.idx);
			goto err;
		}
	} else {
		goto err;
	}
	/* Save state for next packet */
	mscc_ifh_copy_iterator(&priv->curiter, &next);
	pr_debug("%s:%d %s: skb: len: %d, data: 0x%px, used_blocks: %u\n", 
		__FILE__, __LINE__, __func__, skb->len, skb->data, used_blocks);

	priv->rx_packets++;
	priv->rx_bytes += skb->len;

	pr_debug("%s:%d %s: RX Done: packets: %lu, bytes: %lu, allocations: %d\n", 
		__FILE__, __LINE__, __func__, 
		priv->rx_packets,
		priv->rx_bytes,
		priv->allocations);

#ifdef DEBUG
	mscc_ifh_dump_byte_array("RxSkb", skb->data, skb->len);
#endif
	netif_rx(skb);

	priv->available_rx_blocks -= used_blocks;
	mscc_ifh_provide_blocks(priv, req->blocks, req->size);
	return;
err:
	pr_err("%s:%d %s: error: %u, [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, 
		result->result, next.req->cookie, next.idx);
	/* TODO: count unreceived bytes/packet */
	/* Save state for next packet */
	mscc_ifh_copy_iterator(&priv->curiter, &next);
}

static int mscc_ifh_receive_blocks(struct mscc_ifh_priv *priv, 
				   int blocks, int size)
{
	struct dma_async_tx_descriptor *txd;
	struct mscc_ifh_request *req;

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	req = mscc_ifh_prepare_rx_request(priv, DMA_FROM_DEVICE,
				       blocks, size);
	if (!req) {
		pr_err("%s:%d %s: no more requests\n",
			__FILE__, __LINE__, __func__);
		return 0;
	}
	priv->available_rx_blocks += blocks;
	if (!priv->curiter.req) {
		mscc_ifh_init_iterator(&priv->curiter, 0, req);
	}
	txd = dmaengine_prep_slave_sg(priv->rxdma, req->sgl, blocks,
				      DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);

	if (!txd) {
		dev_err(priv->dev, "Could not get RX Descriptor\n");
		goto unmap;
	}

	pr_debug("%s:%d %s: txd: 0x%px\n", 
		__FILE__, __LINE__, __func__, txd);
	txd->callback_param = req;
	txd->callback_result = mscc_ifh_receive_blocks_cb;
	req->cookie = dmaengine_submit(txd);
	if (req->cookie < DMA_MIN_COOKIE) {
		dev_err(priv->dev, "Submit failed\n");
		goto unmap;
	}
	pr_debug("%s:%d %s: Submitted: %d\n", __FILE__, __LINE__, __func__, 
		 req->cookie);
	dma_async_issue_pending(priv->rxdma);
	return 1;

unmap:
	pr_err("%s:%d %s: error, close request\n", __FILE__, __LINE__, __func__);
	mscc_ifh_close_request(priv, req, 1, 1);
	return 0;
}

static void mscc_ifh_provide_blocks(struct mscc_ifh_priv *priv, 
				      int blocks, int size)
{
	pr_debug("%s:%d %s: blocks: %u, size: %u\n", 
		__FILE__, __LINE__, __func__, blocks, size);
	while (priv->available_rx_blocks * size < FDMA_REQUEST_THRESHOLD) {
		mscc_ifh_receive_blocks(priv, blocks, size);
	}
	pr_debug("%s:%d %s: available blocks: %d\n", 
		__FILE__, __LINE__, __func__, priv->available_rx_blocks);
}

static void mscc_ifh_transmit_cb(void *data, 
	const struct dmaengine_result *result)
{
	struct mscc_ifh_request *req = data;
	enum dma_status status;
	struct dma_tx_state state;
	struct mscc_ifh_priv *priv;

	pr_debug("%s:%d %s: result: %u\n", 
		__FILE__, __LINE__, __func__, 
		result->result);

	if (!req) {
		return;
	}
	priv = req->priv;
	if (result->result != DMA_TRANS_NOERROR) {
		pr_err("%s:%d %s: error: %u, [C%u]\n", 
			__FILE__, __LINE__, __func__, 
			result->result, req->cookie);
	} else {
		priv->tx_packets++;
		status = dmaengine_tx_status(priv->txdma, req->cookie, &state);
		pr_debug("%s:%d %s: status %d, state: last: %u, used: %u, "
			 "residue: %u\n", 
			__FILE__, __LINE__, __func__, 
			status, state.last, state.used, state.residue);
		/* TODO: req->size should cover the whole thing */
		priv->tx_bytes += req->blocks * req->size;
	}
	mscc_ifh_close_request(priv, req, 1, 0);
	dev_consume_skb_any(req->skb);

	pr_debug("%s:%d %s: TX Done: packets: %lu, bytes: %lu, allocations: %d\n", 
		__FILE__, __LINE__, __func__, 
		priv->tx_packets,
		priv->tx_bytes,
		priv->allocations);
}

static int mscc_ifh_transmit_fdma(struct mscc_ifh_priv *priv, struct sk_buff *skb)
{
	struct dma_async_tx_descriptor *txd;
	struct mscc_ifh_request *req;

	req = mscc_ifh_prepare_tx_request(priv, DMA_TO_DEVICE, skb);
	if (!req) {
		pr_debug("%s:%d %s: request prepare error\n",
			__FILE__, __LINE__, __func__);
		return 0;
	}
	txd = dmaengine_prep_slave_sg(priv->txdma, req->sgl, req->blocks,
				      DMA_MEM_TO_DEV, 0);

	if (!txd) {
		dev_err(priv->dev, "Could not get TX Descriptor\n");
		goto unmap;
	}

	pr_debug("%s:%d %s: txd: 0x%px\n", 
		__FILE__, __LINE__, __func__, txd);
	txd->callback_param = req;
	txd->callback_result = mscc_ifh_transmit_cb;
	req->cookie = dmaengine_submit(txd);
	if (req->cookie < DMA_MIN_COOKIE) {
		dev_err(priv->dev, "Submit failed\n");
		goto unmap;
	}
	pr_debug("%s:%d %s: Submitted: %d\n", 
		 __FILE__, __LINE__, __func__, req->cookie);
	dma_async_issue_pending(priv->txdma);
	return 1;

unmap:
	pr_err("%s:%d %s: error, close request\n", __FILE__, __LINE__, __func__);
	mscc_ifh_close_request(priv, req, 1, 0);
	dev_consume_skb_any(req->skb);
	return 0;
}

static int mscc_ifh_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}


static int mscc_ifh_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static int mscc_ifh_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mscc_ifh_priv *priv = netdev_priv(dev);

	pr_debug("%s:%d %s: %s: Transmit %d bytes: 0x%px\n",
		 __FILE__, __LINE__, __func__,
		 dev->name, skb->len, skb->data);

#ifdef DEBUG
	mscc_ifh_dump_byte_array("TxSkb", skb->data, skb->len);
#endif
	if (use_fdma(priv)) {
		mscc_ifh_transmit_fdma(priv, skb);
	} else {
		mscc_ifh_transmit_register_based(priv, skb);
	}

	skb_tx_timestamp(skb);
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;
}

static int mscc_ifh_change_mtu(struct net_device *dev, int new_mtu)
{
	struct mscc_ifh_priv *priv = netdev_priv(dev);

	if (new_mtu < (RX_MTU_MIN + ETH_VLAN_TAGSZ + ETH_FCS_LEN + 
		       priv->config->ifh_len) || new_mtu > IF_BUFSIZE_JUMBO) {
		return -EINVAL;
	}
	dev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops mscc_ifh_netdev_ops = {
	.ndo_open            = mscc_ifh_open,
	.ndo_stop            = mscc_ifh_close,
	.ndo_start_xmit      = mscc_ifh_start_xmit,
	.ndo_change_mtu      = mscc_ifh_change_mtu,
	.ndo_validate_addr   = eth_validate_addr,
	.ndo_set_mac_address = eth_mac_addr,
};

static struct net_device *ifhdev_create(struct platform_device *pdev)
{
	struct mscc_ifh_priv *priv;
	struct net_device *dev;

	if ((dev = devm_alloc_etherdev(&pdev->dev, sizeof(*priv))) == NULL) {
		return NULL;
	}

	dev->netdev_ops = &mscc_ifh_netdev_ops;
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof(*priv));
	priv->netdev = dev; /* Backlink */
	priv->dev = &pdev->dev;
	priv->config = device_get_match_data(&pdev->dev);
	/* This device adds no MAC header - it must be part of data */
	// dev->hard_header_len = dev->min_header_len = 0;

	eth_hw_addr_random(dev);
	memset(&dev->broadcast[0], 0xff, 6);

	/* Set arbitrarily high for direct injection (not rx) */
	dev->mtu = IF_BUFSIZE_JUMBO;

	spin_lock_init(&priv->lock);

	return dev;
}

static int mscc_ifh_probe(struct platform_device *pdev)
{
	struct mscc_ifh_priv *priv;
	struct net_device *dev;
	int idx;
	int ret;

	if ((dev = ifhdev_create(pdev)) == NULL) {
		dev_info(&pdev->dev, "Cannot create netdevice\n");
		return -ENOMEM;
	}
	strcpy(dev->name, DEVNAME);

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);
	priv = netdev_priv(dev);

	/* Request TX and RX DMA Channels */
	priv->txdma = dma_request_chan(priv->dev, "tx");
	priv->rxdma = dma_request_chan(priv->dev, "rx");
	if (IS_ERR(priv->txdma) || IS_ERR(priv->rxdma)) {
		if (!IS_ERR(priv->rxdma))
			dma_release_channel(priv->rxdma);
		if (!IS_ERR(priv->txdma))
			dma_release_channel(priv->txdma);
		priv->rxdma = NULL;
		priv->txdma = NULL;
		/* TODO: Use register access */
		mscc_ifh_module_retry++;
		if (mscc_ifh_module_retry == 3) {
			struct resource *res;
			int irq;

			priv->inj_port = 0; /* chipport 65 on Fireant */
			res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
			priv->io_addr = 
				ioremap(res->start, res->end - res->start + 1);

			mscc_ifh_register_based_injection(priv);

			tasklet_init(&priv->tasklet, mscc_ifh_xtr_tasklet,
				     (unsigned long)priv);

			irq = platform_get_irq_byname(pdev, "xtr");
			if (irq < 0) {
				dev_warn(priv->dev, "No IRQ\n");
				return -ENODEV;
			}

			ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
				mscc_ifh_xtr_irq_handler, IRQF_ONESHOT,
				"frame extraction", priv);
			if (ret) {
				dev_warn(priv->dev, "No IRQ thread\n");
				return ret;
			}
			dev_warn(priv->dev, 
				 "No FDMA support: Going register based\n");
		} else {
			dev_warn(priv->dev, 
				 "No FDMA support: retry later: %d\n", 
				 mscc_ifh_module_retry);
			return -EPROBE_DEFER;
		}
	} else {
		dev_info(priv->dev, "Requested TX & RX DMA channels\n");
	}

	INIT_LIST_HEAD(&priv->free_reqs);
	INIT_LIST_HEAD(&priv->rx_reqs);
	INIT_LIST_HEAD(&priv->tx_reqs);
	if (use_fdma(priv)) {
		for (idx = 0; idx < FDMA_REQUEST_MAX; ++idx) {
			struct mscc_ifh_request *req =
				devm_kzalloc(priv->dev, sizeof(*req), GFP_KERNEL);
			if (!req) {
				goto err_list;
			}
			list_add(&req->node, &priv->free_reqs);
		}

		mscc_ifh_provide_blocks(priv, FDMA_XTR_BUFFER_COUNT, 
					FDMA_XTR_BUFFER_SIZE);
	}

	dev->needed_headroom = priv->config->ifh_encap_len + priv->config->ifh_len;
	dev->needed_tailroom = ETH_FCS_LEN;

	ret = register_netdev(dev);
	if (ret < 0) {
		dev_err(priv->dev, "Cannot register netdevice: %d\n", ret);
		free_netdev(dev);
		return ret;
	}

	return 0;

err_list:
	dev_err(priv->dev, "Request allocation error\n");
	return -ENOMEM;
}

static int mscc_ifh_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct mscc_ifh_priv *priv = netdev_priv(dev);

	dev_info(priv->dev, "removing\n");

	if (use_fdma(priv)) {
		dma_release_channel(priv->rxdma);
		dma_release_channel(priv->txdma);
	} else {
		iounmap(priv->io_addr);
	}

	unregister_netdev(dev);
	free_netdev(dev);
	return 0;
}

static const struct fdma_config fireant_data = {
	.first_cpu_port = 65,
	.ifh_id = 0xb,
	.ifh_len = 36,
	.ifh_encap_len = 16,
};

static const struct of_device_id mscc_ifh_match[] = {
	{ .compatible = "mscc,vsc7558-fdma-ifh", .data = &fireant_data },
	{},
};
MODULE_DEVICE_TABLE(of, mscc_ifh_match);

static struct platform_driver mscc_ifh_driver = {
	.probe = mscc_ifh_probe,
	.remove = mscc_ifh_remove,
	.driver = {
		.name = "mscc-fdma-ifh",
		.of_match_table = mscc_ifh_match,
	},
};

module_platform_driver(mscc_ifh_driver);

MODULE_AUTHOR("Steen Hegelund <steen.hegelund@microchip.com>");
MODULE_DESCRIPTION("Microsemi FDMA IFH driver");
MODULE_LICENSE("Dual MIT/GPL");
