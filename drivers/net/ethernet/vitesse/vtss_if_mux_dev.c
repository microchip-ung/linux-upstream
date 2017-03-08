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

#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/rtnetlink.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>
#include <linux/etherdevice.h>
#include <net/switchdev.h>

#include "vtss_if_mux.h"

static const unsigned char hdr_tmpl[IFH_ENCAP_LEN + 4] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,    // Ethernet encapslation
    0xfe, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x88, 0x80, 0x00, IFH_ID,
#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    0x00, 0x00, 0x28, 0x0f, 0x00, 0x40, 0x00, 0x01,
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
    0x00, 0x2e, 0xe5, 0x41, 0x16, 0x58, 0x02, 0x00,
    0x00, 0x00, 0x28, 0x0f, 0x00, 0x40, 0x00, 0x01,
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x01,
    0xa8, 0x00, 0x00, 0x00,
#else
#error Architecture not supported
#endif
    0x00, 0x00, 0x00, 0x00    // Vlan tag placeholder
};

static int internal_dev_open(struct net_device *netdev) {
    unsigned int vlan_id = vtss_if_mux_dev_priv(netdev)->vlan_id;;

    printk(KERN_INFO "internal_dev_open, dev: %p, vlan_id: %u\n", netdev, vlan_id);
    if (vtss_if_mux_vlan_up[vlan_id]) {
        netif_carrier_on(netdev);
    } else {
        netif_carrier_off(netdev);
    }
    return 0;
}

static int internal_dev_xmit(struct sk_buff *skb, struct net_device *netdev) {
    int tx_ok = 1;
    unsigned char *hdr;
    unsigned int i, ret, len, vlan_id;
    struct vtss_if_mux_pcpu_stats *stats;
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(netdev);

    if (!vtss_if_mux_parent_dev) {
        tx_ok = 0;
        //printk(KERN_INFO "No parent device\n");
        goto DO_CNT;
    }

    if (skb_headroom(skb) < sizeof(hdr_tmpl)) {
        tx_ok = 0;
        printk(KERN_INFO "Not enough room for VTSS-header: %u\n",
               skb_headroom(skb));
        goto DO_CNT;
    }

    vlan_id = vtss_if_mux_dev_priv(netdev)->vlan_id;

    // Make room for the VTSS-IFH
    hdr = skb_push(skb, sizeof(hdr_tmpl));

    // Write the template header
    memcpy(hdr, hdr_tmpl, sizeof(hdr_tmpl));

    // move the da/sa to make room for vlan tag (dummy tag is last in temoplate)
    for (i = 0; i < 12; ++i) {
        skb->data[i + IFH_ENCAP_LEN] = skb->data[i + IFH_ENCAP_LEN + 4];
    }

    // Write the vlan tag
    skb->data[IFH_ENCAP_LEN + 12 + 0] = 0x81;
    skb->data[IFH_ENCAP_LEN + 12 + 1] = 0x00;
    skb->data[IFH_ENCAP_LEN + 12 + 2] = (vlan_id >> 8) & 0x0f;
    skb->data[IFH_ENCAP_LEN + 12 + 3] = vlan_id & 0xff;

    // update the placement of the mac-header
    skb->mac_header = IFH_ENCAP_LEN;

    len = skb->len;
    skb->dev = vtss_if_mux_parent_dev;

#if 0
    printk(KERN_INFO "TX %u bytes on vlan %u\n", len, vlan_id);
    print_hex_dump(KERN_INFO, "TX: ", DUMP_PREFIX_OFFSET, 16, 1,
                   skb->data, skb->len, false);
#endif

    ret = dev_queue_xmit(skb);


DO_CNT:
    stats = this_cpu_ptr(priv->vtss_if_mux_pcpu_stats);
    u64_stats_update_begin(&stats->syncp);
    if (likely(tx_ok)) {
        if (likely(ret == NET_XMIT_SUCCESS || ret == NET_XMIT_CN)) {
            stats->tx_packets++;
            stats->tx_bytes += len;
        } else {
            stats->tx_dropped++;
        }
    } else {
        stats->tx_errors++;
    }
    u64_stats_update_end(&stats->syncp);

    return 0;
}

static struct rtnl_link_stats64 *internal_dev_get_stats(
        struct net_device *netdev, struct rtnl_link_stats64 *stats) {
    if (vtss_if_mux_dev_priv(netdev)->vtss_if_mux_pcpu_stats) {
        int i;
        struct vtss_if_mux_pcpu_stats *p;

        for_each_possible_cpu(i) {
            u64 rx_packets, rx_bytes, rx_multicast, rx_dropped, rx_errors;
            u64 tx_packets, tx_bytes,               tx_dropped, tx_errors;

            unsigned int start;
            p = per_cpu_ptr(
                    vtss_if_mux_dev_priv(netdev)->vtss_if_mux_pcpu_stats, i);
            do {
                start = u64_stats_fetch_begin(&p->syncp);
                rx_packets   = p->rx_packets;
                rx_bytes     = p->rx_bytes;
                rx_multicast = p->rx_multicast;
                rx_dropped   = p->rx_dropped;
                rx_errors    = p->rx_errors;
                tx_packets   = p->tx_packets;
                tx_bytes     = p->tx_bytes;
                tx_dropped   = p->tx_dropped;
                tx_errors    = p->tx_errors;
            } while (u64_stats_fetch_retry(&p->syncp, start));

            stats->rx_packets    += rx_packets;
            stats->rx_bytes      += rx_bytes;
            stats->multicast     += rx_multicast;
            stats->rx_dropped    += rx_dropped;
            stats->rx_errors     += rx_errors;
            stats->tx_packets    += tx_packets;
            stats->tx_bytes      += tx_bytes;
            stats->tx_dropped    += tx_dropped;
            stats->tx_errors     += tx_errors;
        }
    }

    return stats;
}

static int internal_dev_change_mtu(struct net_device *netdev, int new_mtu)
{
    printk(KERN_INFO "internal_dev_change_mtu\n");
    if (new_mtu < 68)
        return -EINVAL;

    netdev->mtu = new_mtu;
    return 0;
}

static void internal_dev_getinfo(struct net_device *netdev,
                                 struct ethtool_drvinfo *info) { }

static int internal_dev_init(struct net_device *dev) {
    int i;
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(dev);

    netif_carrier_off(dev);

    priv->fdb_dump_pending = false;

    priv->vtss_if_mux_pcpu_stats =
            alloc_percpu(struct vtss_if_mux_pcpu_stats);

    if (!priv->vtss_if_mux_pcpu_stats)
        return -ENOMEM;

    for_each_possible_cpu(i) {
        struct vtss_if_mux_pcpu_stats *s;
        s = per_cpu_ptr(priv->vtss_if_mux_pcpu_stats, i);
        u64_stats_init(&s->syncp);
    }

    return 0;
}

static void internal_dev_uninit(struct net_device *dev) { }

static void rt_notify_task(struct work_struct *work) {
    int i;
    struct net_device *dev = 0;

    rtnl_lock();

    for (i = 0; i < VLAN_N_VID; ++i) {
        dev = vtss_if_mux_vlan_net_dev[i];
        if (!dev) {
            continue;
        }

        vtss_if_mux_rt_notify(dev);
    }

    rtnl_unlock();
}

static DECLARE_WORK(rt_notify_work, rt_notify_task);

void vtss_if_mux_dev_uninit() {
    cancel_work_sync(&rt_notify_work);
}


static void internal_dev_set_rx_mode(struct net_device *dev) {
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(dev);

    if (!priv->fdb_dump_pending) {
        priv->fdb_dump_pending = true;
        schedule_work(&rt_notify_work);
    }
}

static void internal_dev_destructor(struct net_device *dev) {
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(dev);

    free_percpu(priv->vtss_if_mux_pcpu_stats);
    priv->vtss_if_mux_pcpu_stats = NULL;
    free_netdev(dev);
}

static int internal_dev_fdb_dump(struct sk_buff *skb,
                                 struct netlink_callback *cb,
                                 struct net_device *dev,
                                 struct net_device *filter_dev,
                                 int *idx) {
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(dev);

    // Re-arm the notification mechanism
    priv->fdb_dump_pending = false;
    return ndo_dflt_fdb_dump(skb, cb, dev, filter_dev, idx);
}

static int internal_dev_set_mac_address(struct net_device *dev, void *p) {
    struct sockaddr *addr = p;

    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;

    memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);
    return 0;
}

static const struct net_device_ops internal_dev_netdev_ops = {
    .ndo_init                  = internal_dev_init,
    .ndo_uninit                = internal_dev_uninit,
    .ndo_open                  = internal_dev_open,
    //.ndo_stop: Not used
    .ndo_start_xmit            = internal_dev_xmit,
    .ndo_change_mtu            = internal_dev_change_mtu,
    .ndo_get_stats64           = internal_dev_get_stats,
    .ndo_set_rx_mode           = internal_dev_set_rx_mode,
    .ndo_set_mac_address       = internal_dev_set_mac_address,
    .ndo_fdb_dump              = internal_dev_fdb_dump,
};

static const struct ethtool_ops internal_dev_ethtool_ops = {
    .get_drvinfo = internal_dev_getinfo,
    .get_link = ethtool_op_get_link,
};

void vtss_if_mux_setup(struct net_device *netdev) {
    ether_setup(netdev);
    netdev->netdev_ops = &internal_dev_netdev_ops;

    // We add injection header plus 4 byte VLAN tag
    netdev->needed_headroom = sizeof(hdr_tmpl);

    netdev->destructor = internal_dev_destructor;
    netdev->priv_flags |= IFF_UNICAST_FLT;
    netdev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
    netdev->priv_flags &= ~(IFF_XMIT_DST_RELEASE | IFF_TX_SKB_SHARING);
    netdev->tx_queue_len = 0;
}
