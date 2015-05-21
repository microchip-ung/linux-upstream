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

//#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>

#include "vtss_ufdma_api.h"

#if defined(CONFIG_VTSS_VCOREIII_SERVAL1)
#include <asm/mach-serval/hardware.h>
#define IFH_ID   0x05
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2) || defined(CONFIG_VTSS_VCOREIII_SERVALT)
#include <asm/mach-jaguar2/hardware.h>
#define IFH_ID   0x07
#elif defined(CONFIG_VTSS_VCOREIII_LUTON26)
#include <asm/mach-vcoreiii/hardware.h>
#define IFH_ID   0x01  /* No IFH_ID in Luton26, madeup 0x01 */
#else
#error Invalid architecture type
#endif

static u8 ifh_encap [] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                           0x88, 0x80, 0x00, IFH_ID };

#define PROTO_B0 12
#define PROTO_B1 13

#define DRV_NAME        "vc3fdma"
#define DRV_VERSION     "0.30"
#define DRV_RELDATE     "2015/21/05"

struct vc3fdma_device {
    struct net_device *dev;
};

#define RX_BUFFERS 16

typedef struct {
    struct sk_buff        *skb;               // Associated SKB (or NULL)
    void                  *ufdma_bufstate;    // uFDMA buffer state (private)
} buffer_desc;

/* Information that need to be kept for each board. */
struct vc3fdma_private {
    spinlock_t lock;        /* NIC xmit lock */
    struct net_device *dev;
    struct napi_struct napi;
    vtss_ufdma_platform_driver_t *driver;
    // Rx state
    int rx_work_done;
    size_t rx_bufsize;
    // Rx buffer pool
    buffer_desc rx_desc[RX_BUFFERS];
};

// rx buffer load
static void rx_buffers_refresh(struct net_device *dev)
{
    struct vc3fdma_private *lp = netdev_priv(dev);
    vtss_ufdma_buf_dscr_t buf_dscr;
    int i, rc;
    
    for (i = 0; i < RX_BUFFERS; i++) {
        if (lp->rx_desc[i].skb == NULL) {
            struct sk_buff *skb = netdev_alloc_skb(dev, lp->rx_bufsize);
            buf_dscr.context        = &lp->rx_desc[i];
            buf_dscr.buf            = skb->data;
            buf_dscr.buf_size_bytes = ETH_FRAME_LEN + ETH_FCS_LEN + lp->driver->props.rx_ifh_size_bytes;
            buf_dscr.buf_state      = lp->rx_desc[i].ufdma_bufstate;
            if((rc = lp->driver->rx_buf_add(lp->driver, &buf_dscr))) {
                printk(KERN_ERR "uFDMA rx_buf_add error: %s\n", lp->driver->error_txt(lp->driver, rc));
                consume_skb(skb);
            } else {
                lp->rx_desc[i].skb = skb;
            }
        }
    }
}

/**
 * uFDMA support functions
 */
static void RX_callback(vtss_ufdma_platform_driver_t *unused, vtss_ufdma_buf_dscr_t *buf_dscr)
{
    buffer_desc *bd = buf_dscr->context;
    struct sk_buff *skb = bd->skb;
    struct net_device *dev = skb->dev;
    struct vc3fdma_private *lp = netdev_priv(dev);

    // First add the frame as received by uFDMA (includes IFH *and* FCS)
    skb_put(skb, buf_dscr->frm_length_bytes);
    // Add IFH ethernet encapsulation header
    memcpy(skb_push(skb, sizeof(ifh_encap)), ifh_encap, sizeof(ifh_encap));

#ifdef DEBUG
    print_hex_dump_bytes("rx ", DUMP_PREFIX_NONE, skb->data, skb->len);
#endif
    skb->protocol = eth_type_trans(skb, dev);
    dev_dbg(&dev->dev, "Deliver skb %p, len %d, proto 0x%x\n", skb, skb->len, skb->protocol);

    dev->stats.rx_packets++;
    dev->stats.rx_bytes += skb->len;

    /* Pass the packet to upper layers */
    netif_rx(skb);

    lp->rx_work_done++;

    bd->skb = NULL;    // Skb gone
}

static void TX_callback(vtss_ufdma_platform_driver_t *unused, vtss_ufdma_buf_dscr_t *buf_dscr)
{
    struct sk_buff *skb = buf_dscr->context;
    consume_skb(skb);
}

/****************************************************************************/
// CX_cache_flush()
/****************************************************************************/
static void CX_cache_flush(void *virt_addr, unsigned int bytes)
{
    dma_cache_sync(NULL, virt_addr, bytes, DMA_TO_DEVICE);
}

/****************************************************************************/
// CX_cache_invalidate()
/****************************************************************************/
static void CX_cache_invalidate(void *virt_addr, unsigned int bytes)
{
    dma_cache_sync(NULL, virt_addr, bytes, DMA_FROM_DEVICE);
}

/****************************************************************************/
// CX_virt_to_phys()
/****************************************************************************/
static void *CX_virt_to_phys(void *virt_addr)
{
    return (void*)virt_to_phys(virt_addr);
}

static void CX_trace_printf(vtss_ufdma_trace_layer_t layer, 
                            vtss_ufdma_trace_group_t group, 
                            vtss_ufdma_trace_level_t level, 
                            const char *file, 
                            const int line, 
                            const char *function, 
                            const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    printk("%s:%d:%s: ", file, line, function);
    vprintk(fmt, args);
    va_end(args);
}

static void CX_trace_hex_dump(vtss_ufdma_trace_layer_t layer,
                              vtss_ufdma_trace_group_t group,
                              vtss_ufdma_trace_level_t level,
                              const char *file, 
                              const int line, 
                              const char *function,
                              const unsigned char *byte_p, int byte_cnt)
{
    char loghead[64];
    snprintf(loghead, sizeof(loghead), "%s:%d:%s: ", file, line, function);
    print_hex_dump_bytes(loghead, DUMP_PREFIX_OFFSET, byte_p, byte_cnt);
}

/****************************************************************************/
// CX_timestamp()
/****************************************************************************/
static unsigned long long CX_timestamp(void)
{
    return jiffies;
}

// Register window dword boundaries
#define MEM1_BEG (0) 
#define MEM1_END (MEM1_BEG + (VTSS_IO_ORIGIN1_SIZE/4)) 
#define MEM2_BEG ((VTSS_IO_ORIGIN2_OFFSET-VTSS_IO_ORIGIN1_OFFSET)/4) 
#define MEM2_END (MEM2_BEG + (VTSS_IO_ORIGIN2_SIZE/4)) 

// Register helper
static volatile unsigned int *get_reg(unsigned int chip_no, unsigned int addr)
{
    if (addr >= MEM1_BEG && addr < MEM1_END) {
        return &((volatile unsigned int *)map_base_switch)[addr - MEM1_BEG];
    } else if (addr >= MEM2_BEG && addr < MEM2_END) {
        return &((volatile unsigned int *)map_base_cpu)[addr - MEM2_BEG];
    }
    return NULL;
}

/****************************************************************************/
// CX_reg_read()
/****************************************************************************/
static unsigned int CX_reg_read(unsigned int chip_no, unsigned int addr)
{
    volatile unsigned int *regptr = get_reg(chip_no, addr);
    if(regptr) {
        return *regptr;
    }
    printk("%s: Illegal address referenced: address = %d:0x%08x\n", __FUNCTION__, chip_no, addr);
    BUG();
    return -1;
}

/****************************************************************************/
// CX_reg_write()
/****************************************************************************/
static void CX_reg_write(unsigned int chip_no, unsigned int addr, unsigned int value)
{
    volatile unsigned int *regptr = get_reg(chip_no, addr);
    if(regptr) {
        *regptr = value;
        return;
    }
    printk("%s: Illegal address referenced: address = %d:0x%08x\n", __FUNCTION__, chip_no, addr);
    BUG();
}

static int vc3fdma_poll(struct napi_struct *napi, int budget)
{
    struct vc3fdma_private *lp =
            container_of(napi, struct vc3fdma_private, napi);
    struct net_device *dev = lp->dev;

    // Ensure uFDMA driver calls are serialized
    spin_lock(&lp->lock);

    dev_dbg(&dev->dev, "%s - budget %d\n", __FUNCTION__, budget);
    lp->rx_work_done = 0;

    lp->driver->poll(lp->driver);

    // Replenish RX buffers
    rx_buffers_refresh(dev);

    spin_unlock(&lp->lock);

    if (lp->rx_work_done < budget) {
        napi_complete(napi);
        enable_irq(dev->irq);
    }
    return lp->rx_work_done;
}

/* Ethernet DMA interrupt */
static irqreturn_t vc3fdma_dma_interrupt(int irq, void *dev_id)
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
    int i, ret = 0;
    unsigned int want, alloc_size;

    want = ETH_FRAME_LEN + dev->hard_header_len + ETH_FCS_LEN + lp->driver->props.rx_ifh_size_bytes;
    if ((i = lp->driver->rx_buf_alloc_size_get(lp->driver, want, &alloc_size))) {
        printk(KERN_ERR "uFDMA rx_buf_alloc_size_get error: %s\n", lp->driver->error_txt(lp->driver, i));
        return -ENOMEM;
    }
    lp->rx_bufsize = alloc_size;   // Add buffer state data

    spin_lock(&lp->lock);
    rx_buffers_refresh(dev);
    spin_unlock(&lp->lock);

    /* Initialize */
    netif_start_queue(dev);
    if(dev->irq) {
        napi_enable(&lp->napi);
        /* Install the interrupt handler */
        ret = request_irq(dev->irq, vc3fdma_dma_interrupt, 0, DRV_NAME, dev);
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
    vtss_ufdma_buf_dscr_t tbd;
    int rc;

    dev_dbg(&dev->dev, "%s: Transmit %d bytes @ %p\n", dev->name, skb->len, skb->data);

    // Check for proper encapsulation
    if(skb->data[PROTO_B0] != ifh_encap[PROTO_B0] ||
       skb->data[PROTO_B1] != ifh_encap[PROTO_B1]) {
        dev_dbg(&dev->dev, "Wrong encapsulation - dropping %d bytes @ %p (%02x:%02x)\n", 
                skb->len, skb->data, skb->data[PROTO_B0], skb->data[PROTO_B1]);
        dev->stats.tx_dropped++;
        kfree_skb(skb);
        return NETDEV_TX_OK;
    }

    // Transmit - first loose encap, leave SKB pointing at the IFH
    skb_pull_inline(skb, sizeof(ifh_encap));

    spin_lock_irqsave(&lp->lock, flags);
    if (skb_tailroom(skb) >= lp->driver->props.buf_state_size_bytes) {
        memset(&tbd, 0, sizeof(tbd));
        tbd.buf = skb->data;
        tbd.buf_size_bytes = skb->len + ETH_FCS_LEN;    // Include dummy FCS bytes
        tbd.context = skb;
        tbd.buf_state = skb->tail;
        // Start Tx
        dev->trans_start = jiffies;
        if ((rc = lp->driver->tx(lp->driver, &tbd))) {
            printk(KERN_ERR "uFDMA transmit error: %s\n", lp->driver->error_txt(lp->driver, rc));
            dev->stats.tx_dropped++;
            kfree_skb(skb);
        } else {
            dev->stats.tx_packets++;
            dev->stats.tx_bytes += skb->len;
        }
    } else {
        printk(KERN_ERR "tx: skb(%d) tailroom too small: %d - need %d\n", 
               skb->len, skb_tailroom(skb), lp->driver->props.buf_state_size_bytes);
        dev->stats.tx_dropped++;
        kfree_skb(skb);
    }
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
    vtss_ufdma_init_conf_t init_conf;
    size_t state_len;
    int i, rc;

    dev = alloc_etherdev(sizeof(struct vc3fdma_private));
    if (!dev)
        return NULL;

    dev->netdev_ops = &vc3fdma_netdev_ops;
    lp = netdev_priv(dev);
    memset(lp, 0, sizeof(*lp));
    lp->dev = dev;

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    lp->driver = &vtss_ufdma_platform_driver_luton26;
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
    lp->driver = &vtss_ufdma_platform_driver_serval;
#elif defined(CONFIG_VTSS_VCOREIII_SERVALT)
    lp->driver = &vtss_ufdma_platform_driver_servalt;
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2)
    lp->driver = &vtss_ufdma_platform_driver_jaguar2ab;
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2C)
    lp->driver = &vtss_ufdma_platform_driver_jaguar2c;
#error "Unsupported platform"
#endif

    // Initialize uFDMA
    memset(&init_conf, 0, sizeof(init_conf));

    init_conf.rx_callback      = RX_callback;
    init_conf.tx_callback      = TX_callback;
    init_conf.cache_flush      = CX_cache_flush;
    init_conf.cache_invalidate = CX_cache_invalidate;
    init_conf.virt_to_phys     = CX_virt_to_phys;
    init_conf.trace_printf     = CX_trace_printf;
    init_conf.trace_hex_dump   = CX_trace_hex_dump;
    init_conf.timestamp        = CX_timestamp;
    init_conf.reg_read         = CX_reg_read;
    init_conf.reg_write        = CX_reg_write;
#if defined(CONFIG_CPU_BIG_ENDIAN)
    init_conf.big_endian       = true;
#endif

    state_len = lp->driver->props.ufdma_state_size_bytes + (RX_BUFFERS * lp->driver->props.buf_state_size_bytes);
    if ((lp->driver->state = kmalloc(state_len, GFP_KERNEL)) == NULL) {
        printk(KERN_ERR "Out of memory trying to allocate %u bytes\n", state_len);
        free_netdev(dev);
        return NULL;
    }

    // Initialize uFDMA
    memset(lp->driver->state, 0, lp->driver->props.ufdma_state_size_bytes);
    if((rc = lp->driver->init(lp->driver, &init_conf))) {
        printk(KERN_ERR "uFDMA Initialization error: %d\n", rc);
        kfree(lp->driver->state);
        free_netdev(dev);
        return NULL;
    }

    // Chop up per-buffer uFDMA state
    for (i = 0; i < RX_BUFFERS; i++) {
        lp->rx_desc[i].ufdma_bufstate = 
                lp->driver->state + lp->driver->props.ufdma_state_size_bytes +
                (i * lp->driver->props.buf_state_size_bytes);
    }

    // Reserve TX skb state
    dev->needed_tailroom = lp->driver->props.buf_state_size_bytes;

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

        dev->irq = FDMA_IRQ;

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
