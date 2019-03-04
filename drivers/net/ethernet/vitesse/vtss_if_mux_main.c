/* Copyright (c) 2015 Microsemi Corporation

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

/*
 *  MicroSemi Switch Software.
 *
 */

#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/rtnetlink.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include "vtss_if_mux.h"

struct net_device *vtss_if_mux_parent_dev;
struct net_device *vtss_if_mux_vlan_net_dev[VLAN_N_VID];
struct net_device *vtss_if_mux_port_net_dev[VTSS_IF_MUX_PORT_CNT];
int vtss_if_mux_vlan_up[VLAN_N_VID];

struct net_device *vtss_if_mux_parent_dev_get(void) {
    int err;

    if (vtss_if_mux_parent_dev)
        return vtss_if_mux_parent_dev;

    vtss_if_mux_parent_dev = dev_get_by_name(&init_net, "vtss.ifh");
    if (!vtss_if_mux_parent_dev) {
        return NULL;
    }

    err = netdev_rx_handler_register(vtss_if_mux_parent_dev,
                                     vtss_if_mux_rx_handler, 0);
    if (err) {
        printk(KERN_INFO "failed to registerer handler\n");
        goto exit_unreg;
    }

    return vtss_if_mux_parent_dev;

exit_unreg:
    netdev_rx_handler_unregister(vtss_if_mux_parent_dev);
    dev_put(vtss_if_mux_parent_dev);
    vtss_if_mux_parent_dev = 0;
    return NULL;
}

rx_handler_result_t vtss_if_mux_rx_handler(struct sk_buff **pskb) {
    struct sk_buff *skb;
    struct sk_buff *skb_new;
    struct vtss_if_mux_pcpu_stats *stats;
    struct net_device *dev;
    unsigned int vid = 0;
    int ether_type_offset, pop;
    u16 *ether_type;
    int rx_ok = 1;
    u32 chip_port, min_size;
    u16 etype;

    if (!pskb || !(*pskb))
        return RX_HANDLER_PASS;

    skb = *pskb;

#if 0
    printk(KERN_INFO "RX %u bytes\n", skb->len);
    print_hex_dump(KERN_INFO, "RX: ", DUMP_PREFIX_OFFSET, 16, 1,
                   skb->data, skb->len, false);
#endif

    // Frame layout:
#if defined(CONFIG_VTSS_VCOREIII_ARCH)
    // 2 bytes IFH_ID, IFH_LEN bytes IFH, original Ethernet frame, FCS
    min_size = 2 + IFH_LEN + ETH_ZLEN + ETH_FCS_LEN;
#else
    // 2 bytes IFH_ID, IFH_LEN bytes IFH, original Ethernet frame w/o FCS
    min_size = 2 + IFH_LEN + ETH_ZLEN;
#endif

    if (skb->len < min_size) {
        // Where should this be counted??
        printk(KERN_ERR "Error: %s#%d: Short frame of %u bytes (minimum expected = %u bytes)\n", __FILE__, __LINE__, skb->len, min_size);
        print_hex_dump(KERN_ERR, "Rx ", DUMP_PREFIX_ADDRESS, 16, 1, skb->data, skb->len, true);
        return RX_HANDLER_PASS;
    }

    if (skb->protocol != htons(0x8880) || skb->data[0] != 0 || skb->data[1] != IFH_ID) {
        // TODO, Where should this be counted??
        printk(KERN_INFO "Not for us: 0x%04x %02hhx %02hhx\n",
               skb->protocol, skb->data[0], skb->data[1]);
        return RX_HANDLER_PASS;
    }

    // IFH: Check SFLOW/ACL discard and parse the VID/port
#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    if ((skb->data[5] & 0x1f) <= 27) {
        /* SFLOW discard */
        return RX_HANDLER_PASS;
    }
    vid = (((skb->data[8] << 8) | skb->data[9]) & 0xfff);
    chip_port = ((skb->data[2] << 8) | skb->data[3]);
    chip_port = ((chip_port >> 3) & 0x1f);
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
    if ((skb->data[13] & 0xf) <= 12) {
        /* SFLOW discard */
        return RX_HANDLER_PASS;
    }
    if (skb->data[13] & 0x20) {
        /* ACL discard (ACL_ID, bit 0) */
        return RX_HANDLER_PASS;
    }
    vid = (((skb->data[16] << 8) | skb->data[17]) & 0xfff);
    chip_port = ((skb->data[12] >> 3) & 0xf);
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
#define IFH_OFF  2  // We have IFH ID first (2 bytes)

    {
        // When using an external CPU along with an NPI port, it may happen that
        // frames we send *switched* from the CPU get back to the NPI port.
        // These frames must be discarded. They can be detected by the
        // IFH.VSTAX.SRC having the following properties (when originally
        // injected with IFH.PIPELINE_ACT = INJ_MASQ = 1):
        //    SRC_ADDR_MODE == 0
        //    SRC_PORT_TYPE == 1
        //    SRC_INTPN     == 15
        u16 vstax_src = ((skb->data[IFH_OFF + 20] & 0xf) << 8) | skb->data[IFH_OFF + 21];
        if (vstax_src == 0x80f) {
            printk(KERN_INFO "VSTAX.SRC = 0x%x. Discarding\n", vstax_src);
            // Returning RX_HANDLER_CONSUMED makes sure not even to forward this
            // on the raw vtss.ifh socket (to the application).
            return RX_HANDLER_CONSUMED;
        }
    }

    if ((skb->data[IFH_OFF + 23] >> 5) & 1) {
        /* SFLOW discard */
        return RX_HANDLER_PASS;
    }
    if (skb->data[IFH_OFF + 9] & 0x10) {
        /* ACL discard (CL_RSLT, bit 1) */
        return RX_HANDLER_PASS;
    }
    if (skb->data[IFH_OFF + 6] & 0x20) {
        /* GEN_IDX_MODE is VSI, translate to VID */
        vid = vtss_if_mux_vsi2vid(((skb->data[IFH_OFF + 5] << 8) | skb->data[IFH_OFF + 6]) >> 6);
    }
    if (vid == 0) {
        vid = (((skb->data[IFH_OFF + 18] << 8) | skb->data[IFH_OFF + 19]) & 0xfff);
    }
    chip_port = ((skb->data[IFH_OFF + 23] << 8) | skb->data[IFH_OFF + 24]);
    chip_port = ((chip_port >> 3) & 0x3f);

    // When using an external CPU along with an NPI port, it may happen that
    // frames we send *directed* from the CPU get back to the NPI port. These
    // frames must be discarded. They can be detected by the IFH.SRC_PORT being
    // that of the CPU.
#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
    // CPU port == 11 on ServalT
    if (chip_port == 11) {
#else
    // CPU port == 53 on JR2
    if (chip_port == 53) {
#endif
        printk(KERN_INFO "IFH.SRC_PORT == 0x%x. Discarding\n", chip_port);
        // Returning RX_HANDLER_CONSUMED makes sure not even to forward this on
        // the raw vtss.ifh socket (to the application).
        return RX_HANDLER_CONSUMED;
    }
#else
#error Invalid architecture type
#endif

    if (vid < 1 || vid >= VLAN_N_VID) {
        return RX_HANDLER_PASS;
    }

    // Port device takes precedence
    dev = vtss_if_mux_port_net_dev[chip_port];

    if (!dev) {
        // Do nothing if we have no dependent device
        dev = vtss_if_mux_vlan_net_dev[vid];
        if (!dev) {
            //printk(KERN_INFO "Discard, no dependent device\n");
            return RX_HANDLER_PASS;
        }
    }

    // VLAN filtering and tag popping
    ether_type_offset = 2 + IFH_LEN + 12; // skip id, ifh and mac addresses
    ether_type = (u16 *)(skb->data + ether_type_offset);
    pop = vtss_if_mux_filter_vlan_pop(vid, chip_port, ntohs(ether_type[0]), ntohs(ether_type[2]));
    if (pop < 0) {
        //printk(KERN_ERR "Discard, chip_port: %u, vid: %u\n", chip_port, vid);
        return RX_HANDLER_PASS;
    }
    etype = ntohs(ether_type[pop/2]);
    if (etype != 0x0800 && etype != 0x0806 && etype != 0x86dd) {
        //printk(KERN_ERR "Discard, non-IP/ARP frame, port: %u, vid: %u\n", chip_port, vid);
        return RX_HANDLER_PASS;
    }
    //printk(KERN_ERR "Pop %u bytes, chip_port: %u, vid: %u\n", pop, chip_port, vid);

    // Apply IP filter
    if (vtss_if_mux_filter_apply(skb, vid, ether_type_offset + pop)) {
        //printk(KERN_INFO "Discard, IP filter");
        return RX_HANDLER_PASS;
    }

    // Check if SMAC matches our own address
    if (ether_addr_equal(skb->data + ether_type_offset - 6, dev->dev_addr)) {
        //printk(KERN_ERR "Discard own SMAC, chip_port: %u, vid: %u\n", chip_port, vid);
        return RX_HANDLER_PASS;
    }

    skb_new = skb_clone(skb, GFP_ATOMIC /* invoked from softirq context */);
    if (!skb_new) {
        rx_ok = 0;
        goto DO_CNT;
    }

    // Front: Discard the VTSS headers (IFH + 2-byte IFH_ID)
    skb_pull_inline(skb_new, IFH_LEN + 2);

#if defined(CONFIG_VTSS_VCOREIII_ARCH)
    // Back: Discard the FCS, since - on Rx - the VC3FDMA driver includes
    // the FCS, because it on some platforms contains meta data (sFlow)
    skb_trim(skb_new, skb_new->len - ETH_FCS_LEN);
#endif

#if 0
    printk(KERN_INFO "RX %u bytes on vlan %u\n", skb_new->len, vid);
    print_hex_dump(KERN_INFO, "RX: ", DUMP_PREFIX_OFFSET, 16, 1,
                   skb_new->data, skb_new->len, false);
#endif

    // Update the position of the MAC header according to the new "data" pointer
    skb_new->protocol = eth_type_trans(skb_new, dev);
    if (pop) {
        skb_pull_inline(skb_new, pop);
        skb_new->protocol = htons(etype);
    }

    // Lower-layer might have a different MAC-address, or the MAC-address of the
    // IFH might not match the encapsulated address. Either case we need to
    // reconsider the pkt_type based on the new DMAC.
    if (ether_addr_equal(eth_hdr(skb_new)->h_dest, dev->dev_addr))
        skb_new->pkt_type = PACKET_HOST;

    // printk(KERN_INFO "protocol: 0x%04hx pkt_type: %i\n",
    //        ntohs(skb_new->protocol), skb_new->pkt_type);
    netif_rx(skb_new);

DO_CNT:
    stats = this_cpu_ptr(vtss_if_mux_dev_priv(dev)->vtss_if_mux_pcpu_stats);
    u64_stats_update_begin(&stats->syncp);
    if (likely(rx_ok)) {
        stats->rx_packets++;
        stats->rx_bytes += skb_new->len;
        if (skb_new->pkt_type == PACKET_MULTICAST)
            stats->rx_multicast++;
    } else {
        stats->rx_errors++;
    }
    u64_stats_update_end(&stats->syncp);

    return RX_HANDLER_PASS;
}

static int dev_notification(struct notifier_block *unused, unsigned long event,
                            void *ptr) {
    struct net_device *dev = netdev_notifier_info_to_dev(ptr);

    if (dev != vtss_if_mux_parent_dev) return NOTIFY_DONE;

    switch (event) {
        case NETDEV_UNREGISTER:
            if (vtss_if_mux_parent_dev) {
                printk(KERN_INFO "Releasing reference to parent device\n");
                netdev_rx_handler_unregister(vtss_if_mux_parent_dev);
                dev_put(vtss_if_mux_parent_dev);
                vtss_if_mux_parent_dev = 0;
            }
            break;

        default:;
    }

    return NOTIFY_DONE;
}

static struct notifier_block dev_notifier_block __read_mostly = {
    .notifier_call = dev_notification,
};

static int __init vtss_if_mux_init_module(void) {
    int err = 0, i;
    vtss_if_mux_parent_dev = 0;

    printk(KERN_INFO "Loading module vtss-if-mux\n");

    for (i = 0; i < VTSS_IF_MUX_PORT_CNT; ++i)
        vtss_if_mux_port_net_dev[i] = 0;

    for (i = 0; i < VLAN_N_VID; ++i)
        vtss_if_mux_vlan_net_dev[i] = 0;

    err = register_netdevice_notifier(&dev_notifier_block);
    if (err < 0) {
        printk(KERN_INFO "Failed: register_netdevice_notifier\n");
        goto exit;
    }

    err = vtss_if_mux_netlink_init();
    if (err < 0) {
        printk(KERN_INFO "Failed: vtss_if_mux_netlink_init\n");
        goto exit_1;
    }

    err = vtss_if_mux_genetlink_init();
    if (err < 0) {
        printk(KERN_INFO "Failed: vtss_if_mux_genetlink_init\n");
        goto exit_2;
    }

    err = vtss_if_mux_dev_init();
    if (err < 0) {
        printk(KERN_INFO "Failed: vtss_if_mux_dev_init\n");
        goto exit_3;
    }

    return 0;

exit_3:
    vtss_if_mux_genetlink_uninit();

exit_2:
    vtss_if_mux_netlink_uninit();

exit_1:
    unregister_netdevice_notifier(&dev_notifier_block);

exit:
    return err;
}

static void __exit vtss_if_mux_exit_module(void) {
    int i;
    struct net_device *dev;

    printk(KERN_INFO "Unloading module vtss-if-mux\n");
    unregister_netdevice_notifier(&dev_notifier_block);
    vtss_if_mux_netlink_uninit();
    vtss_if_mux_genetlink_uninit();
    vtss_if_mux_dev_uninit();

    for (i = 0; i < VTSS_IF_MUX_PORT_CNT; ++i) {
        dev = vtss_if_mux_port_net_dev[i];
        if (dev) {
            printk(KERN_INFO "unreg net device port=%d %p\n", i, dev);
            unregister_netdev(dev);
            free_netdev(dev);
            vtss_if_mux_port_net_dev[i] = 0;
        }
    }

    for (i = 0; i < VLAN_N_VID; ++i) {
        dev = vtss_if_mux_vlan_net_dev[i];
        if (dev) {
            printk(KERN_INFO "unreg net device vlan=%d %p\n", i, dev);
            unregister_netdev(dev);
            free_netdev(dev);
            vtss_if_mux_vlan_net_dev[i] = 0;
        }
    }

    if (vtss_if_mux_parent_dev) {
        printk(KERN_INFO "unreg\n");
        rtnl_lock();
        netdev_rx_handler_unregister(vtss_if_mux_parent_dev);
        rtnl_unlock();

        dev_put(vtss_if_mux_parent_dev);
        vtss_if_mux_parent_dev = 0;
    }
}

module_init(vtss_if_mux_init_module);
module_exit(vtss_if_mux_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Allan W. Nielsen <anielsen@vitesse.com>");
