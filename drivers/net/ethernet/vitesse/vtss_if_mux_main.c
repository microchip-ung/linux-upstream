/*
 *  Vitesse Switch Software.
 *
 *  Copyright (c) 2002-2015 Vitesse Semiconductor Corporation "Vitesse".
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

#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/rtnetlink.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include "vtss_if_mux.h"

struct net_device *vtss_if_mux_parent_dev;
struct net_device *vtss_if_mux_vlan_net_dev[VLAN_N_VID];

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
    int ether_type_offset = 0;
    u16 *ether_type;
    int rx_ok = 1;

    if (!pskb || !(*pskb))
        return RX_HANDLER_PASS;

    skb = *pskb;

    if (skb->len < (ETH_ZLEN + IFH_ID)) {
        // Where should this be counted??
        dev_err(&dev->dev, "Short frame: %u bytes\n", skb->len);
        return RX_HANDLER_PASS;
    }

    if (skb->protocol != htons(0x8880) || skb->data[0] != 0 || skb->data[1] != IFH_ID) {
        // TODO, Where should this be counted??
        printk(KERN_INFO "Not for us: 0x%04x %02hhx %02hhx\n",
               skb->protocol, skb->data[0], skb->data[1]);
        return RX_HANDLER_PASS;
    }

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    // Parse the VID
    vid = (skb->data[8] & 0xf);
    vid <<= 8;
    vid |= skb->data[9];
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
    // Parse the VID
    vid = (skb->data[16] & 0xf);
    vid <<= 8;
    vid |= skb->data[17];
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
#define IFH_OFF  2                 // We have IFH ID first (2 bytes)
#define VLAN_OFF (IFH_OFF + 18)    // Classified VID at byte offset 18(msb)+19(lsb)
    vid = (skb->data[VLAN_OFF] & 0xf);
    vid <<= 8;
    vid |= skb->data[VLAN_OFF+1];
#else
#error Invalid architecture type
#endif

    if (vid < 1 || vid >= VLAN_N_VID) {
        return RX_HANDLER_PASS;
    }

    // Do nothing if we have no dependend device
    dev = vtss_if_mux_vlan_net_dev[vid];
    if (!dev) {
        return RX_HANDLER_PASS;
    }

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    if ((skb->data[13] & 0x1f) <= 27) {
        printk(KERN_ERR "SFLOW PACKET - delete line when tested!"); // TODO
        return RX_HANDLER_PASS;
    }
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
    if ((skb->data[13] & 0xf) <= 12) {
        printk(KERN_ERR "SFLOW PACKET - delete line when tested!"); // TODO
        return RX_HANDLER_PASS;
    }
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
    if ((skb->data[25] >> 5) & 1) {
        printk(KERN_ERR "SFLOW PACKET - delete line when tested!"); // TODO
        return RX_HANDLER_PASS;
    }
#else
#error Invalid architecture type
#endif

    // TODO, needs configuration from user-space to do this correctly
    // Figure out where the ether type of the next protocol is located. This
    // must consum the vlan tags on ports that are configured to allow
    // vlan-tags.
    ether_type_offset = 2 + IFH_LEN; // skip the id and ifh
    ether_type_offset += 12; // skip the two mac addresses
    ether_type = (u16 *)(skb->data + ether_type_offset);
    if (*ether_type == htons(0x8100)) {  // TODO, multiple ether types are supported
        ether_type_offset += 4; // skip the mac header
    }

    if (vtss_if_mux_filter_apply(skb, vid, ether_type_offset)) {
        return RX_HANDLER_PASS;
    }

    skb_new = skb_clone(skb, GFP_KERNEL);
    if (!skb_new) {
        rx_ok = 0;
        goto DO_CNT;
    }

    // Front: Discard the VTSS headers (IFH + 2-byte IFH_ID)
    skb_pull_inline(skb_new, IFH_LEN + 2);

    // Back: Discard the FCS
    skb_trim(skb_new, skb_new->len - ETH_FCS_LEN);

#if 0
    printk(KERN_INFO "RX %u bytes on vlan %u\n", skb_new->len, vid);
    print_hex_dump(KERN_INFO, "RX: ", DUMP_PREFIX_OFFSET, 16, 1,
                   skb_new->data, skb_new->len, false);
#endif

    // TODO, can not use eth_type_trans as we "some-time" needs to consume the
    // vlan tag as well.
    // Updates the new posistion of the MAC header according to the new "data"
    // pointer
    skb_new->protocol = eth_type_trans(skb_new, dev);

    // Lower-layer might have a different MAC-address, or the MAC-address of the
    // IHF might not match the encapsulated address. Either case we need to
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

    return 0;

exit_2:
    vtss_if_mux_netlink_uninit();

exit_1:
    unregister_netdevice_notifier(&dev_notifier_block);

exit:
    return err;
}

static void __exit vtss_if_mux_exit_module(void) {
    int i;

    printk(KERN_INFO "Unloading module vtss-if-mux\n");
    unregister_netdevice_notifier(&dev_notifier_block);
    vtss_if_mux_netlink_uninit();
    vtss_if_mux_genetlink_uninit();

    for (i = 0; i < VLAN_N_VID; ++i) {
        if (vtss_if_mux_vlan_net_dev[i]) {
            printk(KERN_INFO "unreg net device vlan=%d %p\n", i,
                   vtss_if_mux_vlan_net_dev[i]);
            unregister_netdev(vtss_if_mux_vlan_net_dev[i]);
            free_netdev(vtss_if_mux_vlan_net_dev[i]);
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

