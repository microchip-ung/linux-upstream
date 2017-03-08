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


#include <linux/if_vlan.h>
#include <linux/netdevice.h>
#include <linux/u64_stats_sync.h>

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
#define IFH_ID   0x01  /* No IFH_ID in Luton26, madeup 0x01 */
#define IFH_LEN  8
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1_CLASSIC)
#define IFH_ID   0x05
#define IFH_LEN  16
#elif defined(CONFIG_VTSS_VCOREIII_OCELOT)
#define IFH_ID   0x0a
#define IFH_LEN  16
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
#define IFH_ID   0x07
#define IFH_LEN  28
#else
#error Invalid architecture type
#endif
#define IFH_ENCAP_LEN (12+4+IFH_LEN)    // Eth encap + IFH

struct vtss_if_mux_pcpu_stats {
    u64                         rx_packets;
    u64                         rx_bytes;
    u64                         rx_multicast;
    u64                         rx_dropped;
    u64                         rx_errors;

    u64                         tx_packets;
    u64                         tx_bytes;
    u64                         tx_dropped;
    u64                         tx_errors;

    struct u64_stats_sync       syncp;
};

struct vtss_if_mux_dev_priv {
    u16                                     vlan_id;
    struct vtss_if_mux_pcpu_stats __percpu *vtss_if_mux_pcpu_stats;
    bool                                    fdb_dump_pending;
};

static inline struct vtss_if_mux_dev_priv *vtss_if_mux_dev_priv(
        const struct net_device *dev) {
    return netdev_priv(dev);
}

extern bool vtss_if_mux_nl_notify_pending;
extern struct net_device *vtss_if_mux_parent_dev;
extern struct net_device *vtss_if_mux_vlan_net_dev[VLAN_N_VID];
extern int vtss_if_mux_vlan_up[VLAN_N_VID];

void vtss_if_mux_setup(struct net_device *netdev);

int vtss_if_mux_netlink_init(void);
void vtss_if_mux_netlink_uninit(void);
void vtss_if_mux_dev_uninit(void);

int vtss_if_mux_genetlink_init(void);
void vtss_if_mux_genetlink_uninit(void);

rx_handler_result_t vtss_if_mux_rx_handler(struct sk_buff **pskb);
struct net_device *vtss_if_mux_parent_dev_get(void);

void vtss_if_mux_rt_notify(struct net_device *dev);

int vtss_if_mux_filter_apply(struct sk_buff *skb, unsigned int vid, unsigned int
                             ether_type_offset);
int vtss_if_mux_filter_vlan_pop(u32 vid, u32 chip_port, u32 etype_outer, u32 etype_inner);
