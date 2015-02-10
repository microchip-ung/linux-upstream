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

static int internal_dev_open(struct net_device *netdev) {
    printk(KERN_INFO "internal_dev_open\n");
    netif_carrier_on(netdev);
    return 0;
}

static int internal_dev_stop(struct net_device *netdev) {
    printk(KERN_INFO "internal_dev_stop\n");
    netif_carrier_off(netdev);
    return 0;
}

static int internal_dev_xmit(struct sk_buff *skb, struct net_device *netdev) {
    int tx_ok = 1;
    unsigned char *hdr;
    unsigned int i, ret, len, vlan_id;
    struct vtss_if_mux_pcpu_stats *stats;
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(netdev);

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    static const unsigned char hdr_tmpl[] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff,
        0xff, 0xff, 0xff, 0xff, 0x88, 0x80, 0x00, 0x01,
        0x00, 0x00, 0x28, 0x0f, 0x00, 0x40, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00};
#else
    static const unsigned char hdr_tmpl[] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff,
        0xff, 0xff, 0xff, 0xff, 0x88, 0x80, 0x00, 0x05,
        0x00, 0x2e, 0xe5, 0x41, 0x16, 0x58, 0x02, 0x00,
        0x00, 0x00, 0x28, 0x0f, 0x00, 0x40, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00};
#endif

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

    // move the da/sa to make room for vlan tag
#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    for (i = 0; i < 12; ++i) skb->data[i + 24] = skb->data[i + 28];

    // Write the vlan tag
    skb->data[36] = 0x81;
    skb->data[37] = 0x00;
    skb->data[38] = (vlan_id >> 8) & 0x0f;
    skb->data[39] = vlan_id & 0xff;
#else
    for (i = 0; i < 12; ++i) skb->data[i + 32] = skb->data[i + 36];

    // Write the vlan tag
    skb->data[44] = 0x81;
    skb->data[45] = 0x00;
    skb->data[46] = (vlan_id >> 8) & 0x0f;
    skb->data[47] = vlan_id & 0xff;
#endif

    // update the placement of the mac-header
#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    skb->mac_header = 24;
#else
    skb->mac_header = 32;
#endif

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
        u64_stats_init(&stat->syncp);
    }

    return 0;
}

static void internal_dev_uninit(struct net_device *dev) { }

static void internal_dev_set_rx_mode(struct net_device *dev) {
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(dev);

    if (!priv->fdb_dump_pending) {
        vtss_if_mux_rt_notify(dev);
        priv->fdb_dump_pending = true;
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
                                 struct net_device *dev, int idx) {
    struct vtss_if_mux_dev_priv *priv = vtss_if_mux_dev_priv(dev);

    // Re-arm the notification mechanism
    priv->fdb_dump_pending = false;
    return ndo_dflt_fdb_dump(skb, cb, dev, idx);
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
    .ndo_stop                  = internal_dev_stop,
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

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
    netdev->needed_headroom = 28;  // TODO - 24 for injection header and 4 for
                                   // vlan tag
#else
    netdev->needed_headroom = 36;  // TODO - 32 for injection header and 4 for
                                   // vlan tag
#endif

    netdev->destructor = internal_dev_destructor;
    netdev->priv_flags |= IFF_UNICAST_FLT;
    netdev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
    netdev->priv_flags &= ~(IFF_XMIT_DST_RELEASE | IFF_TX_SKB_SHARING);
    netdev->tx_queue_len = 0;
}

