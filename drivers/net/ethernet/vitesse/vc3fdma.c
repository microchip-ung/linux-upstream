/*
 *  Driver for the VTSS Switch FDMA (and register IO)
 *
 *  Copyright 2014 Lars Povlsen <lpovlsen@vitesse.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>

#if defined(CONFIG_VTSS_VCOREIII_SERVAL1)
#include <asm/mach-serval/hardware.h>
#define IFH_SIZE 16
#define IFH_ID   0x05
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2)
#include <asm/mach-jaguar2/hardware.h>
#define IFH_SIZE 28
#define IFH_ID   0x07
#elif defined(CONFIG_VTSS_VCOREIII_LUTON26)
#include <asm/mach-vcoreiii/hardware.h>
#define IFH_SIZE 8
#define IFH_ID   0x01  /* No IFH_ID in Luton26, madeup 0x01 */
#define DO_PADDING
#else
#error Invalid architecture type
#endif
#define IFH_SIZE_WORDS (IFH_SIZE/4)

static u8 ifh_encap [] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                           0x88, 0x80, 0x00, IFH_ID };

#define PROTO_B0 12
#define PROTO_B1 13

#if !defined(VTSS_M_DEVCPU_QS_INJ_INJ_CTRL_SOF)
#define VTSS_M_DEVCPU_QS_INJ_INJ_CTRL_SOF VTSS_F_DEVCPU_QS_INJ_INJ_CTRL_SOF
#endif
#if !defined(VTSS_M_DEVCPU_QS_INJ_INJ_CTRL_EOF)
#define VTSS_M_DEVCPU_QS_INJ_INJ_CTRL_EOF VTSS_F_DEVCPU_QS_INJ_INJ_CTRL_EOF
#endif

#define DRV_NAME        "vc3fdma"
#define DRV_VERSION     "0.13"
#define DRV_RELDATE     "2015/10/02"

struct vc3fdma_device {
    struct net_device *dev;
};

/* Information that need to be kept for each board. */
struct vc3fdma_private {
    spinlock_t lock;        /* NIC xmit lock */
    struct net_device *dev;
    struct napi_struct napi;
};

static void inject_send(struct net_device *dev, 
                        struct sk_buff *skb,
                        const u32 *ifh)
{
    u32 val, count, last, *buf;
    int w, grp = 0, vid = 0;

    /* Indicate SOF */
    writel(VTSS_F_DEVCPU_QS_INJ_INJ_CTRL_GAP_SIZE(1) | VTSS_M_DEVCPU_QS_INJ_INJ_CTRL_SOF,
           VTSS_DEVCPU_QS_INJ_INJ_CTRL(grp));

    // IFH
    for (w = 0; w < IFH_SIZE_WORDS; w++) {
        val = get_unaligned(&ifh[w]);
        writel(val, VTSS_DEVCPU_QS_INJ_INJ_WR(grp));
    }

    count = (round_up(skb->len,4) / 4);
    last = skb->len % 4;
    for (buf = (u32*) skb->data, w = 0; w < count; w++, buf++) {
        if (w == 3 && vid != 0) {
            /* Insert C-tag */
            writel(ntohl((0x8100U << 16) | vid), VTSS_DEVCPU_QS_INJ_INJ_WR(grp));
            w++;
        }
        val = get_unaligned(buf);
        writel(val, VTSS_DEVCPU_QS_INJ_INJ_WR(grp));
    }

#if defined DO_PADDING
    while (w < (60 / 4)) {
        writel(0, VTSS_DEVCPU_QS_INJ_INJ_WR(grp));
        w++;
    }
#endif

    /* Indicate EOF and valid bytes in last word */
    writel(VTSS_F_DEVCPU_QS_INJ_INJ_CTRL_GAP_SIZE(1) |
           VTSS_F_DEVCPU_QS_INJ_INJ_CTRL_VLD_BYTES(skb->len < 60 ? 0 : last) |
           VTSS_M_DEVCPU_QS_INJ_INJ_CTRL_EOF, 
           VTSS_DEVCPU_QS_INJ_INJ_CTRL(grp));

    /* Add dummy CRC */
    writel(0, VTSS_DEVCPU_QS_INJ_INJ_WR(grp));
    w++;
}

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
#define XTR_EOF_0          0x80000000U
#define XTR_EOF_1          0x80000001U
#define XTR_EOF_2          0x80000002U
#define XTR_EOF_3          0x80000003U
#define XTR_PRUNED         0x80000004U
#define XTR_ABORT          0x80000005U
#define XTR_ESCAPE         0x80000006U
#define XTR_NOT_READY      0x80000007U
#define XTR_VALID_BYTES(x) (4 - (((x) >> 24) & 3))

#else
#define XTR_EOF_0          0x00000080U
#define XTR_EOF_1          0x01000080U
#define XTR_EOF_2          0x02000080U
#define XTR_EOF_3          0x03000080U
#define XTR_PRUNED         0x04000080U
#define XTR_ABORT          0x05000080U
#define XTR_ESCAPE         0x06000080U
#define XTR_NOT_READY      0x07000080U
#define XTR_VALID_BYTES(x) (4 - (((x) >> 24) & 3))

#endif

static struct sk_buff *read_xtrgrp(struct net_device *dev, int grp)
{
    u32 val, bytes_valid = 0, done = 0, maxdata = dev->mtu + ETH_HLEN + sizeof(ifh_encap) + IFH_SIZE + ETH_FCS_LEN;
    struct sk_buff *skb = netdev_alloc_skb(dev, maxdata);
    if (skb) {
        u32 total = 0, *ptr;
        memcpy(skb_tail_pointer(skb), ifh_encap, sizeof(ifh_encap));
        skb_put(skb, sizeof(ifh_encap));
        ptr = (u32*) skb_tail_pointer(skb);
        while(!done) {
            val = readl(VTSS_DEVCPU_QS_XTR_XTR_RD(grp));
            switch (val) {
                case XTR_ABORT:
                    /* No accompanying data. */
                    total = bytes_valid = 0;
                    done = 1;
                    break;
                case XTR_EOF_0:
                case XTR_EOF_1:
                case XTR_EOF_2:
                case XTR_EOF_3:
                case XTR_PRUNED:
                    bytes_valid = XTR_VALID_BYTES(val);
                    val = readl(VTSS_DEVCPU_QS_XTR_XTR_RD(grp));
                    if (val == XTR_ESCAPE) {
                        // again
                        val = readl(VTSS_DEVCPU_QS_XTR_XTR_RD(grp));
                    }
                    done = 1;
                    break;
                case XTR_ESCAPE:
                    val = readl(VTSS_DEVCPU_QS_XTR_XTR_RD(grp));
                    bytes_valid = 4;
                    break;
                default:
                    bytes_valid = 4;
            }
            if (bytes_valid && total < maxdata) {
                *ptr++ = val;
                total += bytes_valid;
            }
        }
        if (total) {
            skb_put(skb, total);
            return skb;
        }
    } else {
        // No buffer, have to purge frame
        dev_notice(&dev->dev, "Out of SKBs, prune data\n");
        while (!done) {
            val = readl(VTSS_DEVCPU_QS_XTR_XTR_RD(grp));
            switch (val) {
                case XTR_ABORT:
                case XTR_PRUNED:
                case XTR_EOF_3:
                case XTR_EOF_2:
                case XTR_EOF_1:
                case XTR_EOF_0:
                    val = readl(VTSS_DEVCPU_QS_XTR_XTR_RD(grp));
                    done = 1;        /* Last 1-4 bytes */
                    break;
                case XTR_ESCAPE:
                    val = readl(VTSS_DEVCPU_QS_XTR_XTR_RD(grp)); /* Escaped data */
                    break;
                case XTR_NOT_READY:
                default:
                    break;
            }
        }
    }
    return NULL;
}

static void vc3fdma_receive(struct net_device *dev, 
                            struct sk_buff *skb)
{
    int len = skb->len;

#ifdef DEBUG
    print_hex_dump_bytes(is_eth ? "ifhrx ", DUMP_PREFIX_NONE, skb->data, skb->len);
#endif
    skb->protocol = eth_type_trans(skb, dev);
    dev_dbg(&dev->dev, "Deliver skb %p, len %d, proto 0x%x\n", skb, skb->len, skb->protocol);

    dev->stats.rx_packets++;
    dev->stats.rx_bytes += len;

    /* Pass the packet to upper layers */
    netif_rx(skb);
}

static int vc3fdma_poll(struct napi_struct *napi, int budget)
{
    struct vc3fdma_private *lp =
            container_of(napi, struct vc3fdma_private, napi);
    struct net_device *dev = lp->dev;
    int i, work_done = 0;
    u32 val;

    dev_dbg(&dev->dev, "%s - budget %d\n", __FUNCTION__, budget);
    while(work_done < budget && (val = readl(VTSS_DEVCPU_QS_XTR_XTR_DATA_PRESENT))) {
        dev_dbg(&dev->dev, "Got DP: 0x%08x, work %d\n", val, work_done);
        for (i = 0; i <= 1; i++) {
            if (val & (1 << i)) {
                struct sk_buff *skb = read_xtrgrp(dev, i);
                if(skb) {
                    vc3fdma_receive(dev, skb);
                }
                work_done++;
            }
        }
    }
    if (work_done < budget) {
        napi_complete(napi);
        enable_irq(dev->irq);
    }
    return work_done;
}

/* Ethernet Rx DMA interrupt */
static irqreturn_t vc3fdma_rx_dma_interrupt(int irq, void *dev_id)
{
    struct net_device *dev = dev_id;
    struct vc3fdma_private *lp = netdev_priv(dev);

    disable_irq_nosync(dev->irq);
    napi_schedule(&lp->napi);

    return IRQ_HANDLED;
}

static int vc3fdma_open(struct net_device *dev)
{
    struct vc3fdma_private *lp = netdev_priv(dev);
    int ret = 0;

    /* Initialize */
    netif_start_queue(dev);
    if(dev->irq) {
        napi_enable(&lp->napi);
        /* Install the interrupt handler */
        ret = request_irq(dev->irq, vc3fdma_rx_dma_interrupt, 0, DRV_NAME " Rx", dev);
        if (ret < 0) {
            dev_err(&dev->dev, "Unable to get Rx DMA IRQ %d\n", dev->irq);
        }
    }
    return ret;
}

static int vc3fdma_close(struct net_device *dev)
{
    if(dev->irq) {
        struct vc3fdma_private *lp = netdev_priv(dev);
        disable_irq(dev->irq);
        napi_disable(&lp->napi);
        free_irq(dev->irq, dev);
    }
    return 0;
}

/* transmit packet */
static int vc3fdma_send_packet(struct sk_buff *skb, struct net_device *dev)
{
    struct vc3fdma_private *lp = netdev_priv(dev);
    unsigned long flags;
    const u32 *ifh_ptr;

    dev_dbg(&dev->dev, "%s: Transmit %d bytes @ %p\n", dev->name, skb->len, skb->data);

    // Check for proper encapsulation
    if( skb->data[PROTO_B0] != ifh_encap[PROTO_B0] ||
        skb->data[PROTO_B1] != ifh_encap[PROTO_B1]) {
        dev_dbg(&dev->dev, "Wrong encapsulation - dropping %d bytes @ %p (%02x:%02x)\n", 
                skb->len, skb->data, skb->data[PROTO_B0], skb->data[PROTO_B1]);
        dev->stats.tx_dropped++;
        kfree_skb(skb);
        return NETDEV_TX_OK;
    }
    // Transmit - first loose encap
    skb_pull_inline(skb, sizeof(ifh_encap));
    // Then pull the ifh
    ifh_ptr = (u32*) skb->data;
    skb_pull_inline(skb, IFH_SIZE);

    spin_lock_irqsave(&lp->lock, flags);
    dev->trans_start = jiffies;
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;

    inject_send(dev, skb, ifh_ptr);
    consume_skb(skb);

    spin_unlock_irqrestore(&lp->lock, flags);

    return NETDEV_TX_OK;
}

static const struct net_device_ops vc3fdma_netdev_ops = {
    .ndo_open		 = vc3fdma_open,
    .ndo_stop		 = vc3fdma_close,
    .ndo_start_xmit      = vc3fdma_send_packet,
    .ndo_change_mtu      = eth_change_mtu,
    .ndo_validate_addr	 = eth_validate_addr,
    .ndo_set_mac_address = eth_mac_addr,
};


struct net_device *vc3fdma_create(void)
{
    struct net_device *dev;
    struct vc3fdma_private *lp;

    dev = alloc_etherdev(sizeof(struct vc3fdma_private));
    if (!dev)
        return NULL;

    lp = netdev_priv(dev);
    dev->netdev_ops = &vc3fdma_netdev_ops;
    lp->dev = dev;

    // Set initial, bogus MAC address
    eth_hw_addr_random(dev);
    memset(&dev->broadcast[0], 0xff, 6);

    spin_lock_init(&lp->lock);

    return dev;
}

static int vc3fdma_probe(struct platform_device *pdev)
{
    struct net_device *dev;
    int rc;

    dev = vc3fdma_create();
    if (!dev) {
        rc = -ENOMEM;
    } else {
        struct vc3fdma_private *lp;

        // We're special aka. "vtss.ifh"
        strcpy(dev->name, "vtss.ifh");

        // Hook the devices together netdev <-> pdev
        SET_NETDEV_DEV(dev, &pdev->dev);
        platform_set_drvdata(pdev, dev);

#if defined(XTR_RDY_IRQ)
        dev->irq = XTR_RDY_IRQ;
#elif defined(XTR_RDY0_IRQ)
        dev->irq = XTR_RDY0_IRQ;
#endif

        rc = register_netdev(dev);
        if (rc < 0) {
            dev_err(&dev->dev, "Cannot register net device: %d\n", rc);
            free_netdev(dev);
        } else {
            // Turn on device handling
            lp = netdev_priv(dev);
            netif_napi_add(dev, &lp->napi, vc3fdma_poll, 64);
            enable_irq(dev->irq);

            printk(KERN_INFO "%s: " DRV_NAME "-" DRV_VERSION " " DRV_RELDATE "\n", dev->name);
        }
    }
    return rc;
}

static int vc3fdma_remove(struct platform_device *pdev)
{
    struct net_device *dev = platform_get_drvdata(pdev);

    unregister_netdev(dev);
    free_netdev(dev);

    return 0;
}

static struct platform_driver vc3fdma_driver = {
    .driver.name = "vc3fdma",
    .probe = vc3fdma_probe,
    .remove = vc3fdma_remove,
};

module_platform_driver(vc3fdma_driver);

MODULE_AUTHOR("Lars Povlsen <lpovlsen@vitesse.com>");
MODULE_DESCRIPTION("VCore-III FDMA ethernet interface driver");
MODULE_LICENSE("GPL");
