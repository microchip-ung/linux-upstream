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

#include <net/rtnetlink.h>
#include <linux/if_vlan.h>
#include <linux/netdevice.h>

#include "vtss_if_mux.h"

static const struct nla_policy vtss_if_mux_policy[IFLA_VLAN_MAX + 1] = {
    [IFLA_VLAN_ID] = { .type = NLA_U16 },
};

static int vlan_id_from_ifname(struct nlattr *tb[]) {
    int vlan_id = -1;

    if (!tb[IFLA_IFNAME])
        return -1;

    if (nla_len(tb[IFLA_IFNAME]) > IFNAMSIZ) {
        printk(KERN_INFO "err 1\n");
        return -1;
    }

    if (sscanf(nla_data(tb[IFLA_IFNAME]), "vtss.vlan.%d", &vlan_id) != 1)
        return -1;

    if (vlan_id <= 0 || vlan_id >= VLAN_N_VID)
        return -1;

    return vlan_id;
}

static int vtss_if_mux_validate(struct nlattr *tb[], struct nlattr *data[]) {
    if (tb[IFLA_IFNAME]) {
        if (vlan_id_from_ifname(tb) == -1) return -EINVAL;
    }

    return 0;
}

static int vtss_if_mux_changelink(struct net_device *dev,
                                  struct nlattr *tb[], struct nlattr *data[]) {
    printk(KERN_INFO "vtss_if_mux_changelink\n");

    return 0;
}

static int vtss_if_mux_newlink(struct net *src_net, struct net_device *dev,
                               struct nlattr *tb[], struct nlattr *data[]) {
    int err;
    int vlan_id;
    struct net_device *parent_dev;

    //printk(KERN_INFO "vtss_if_mux_newlink %p\n", dev);
    vlan_id = vlan_id_from_ifname(tb);
    if (vlan_id == -1) {
        printk(KERN_INFO "No vlan_id\n");
        return -ENODEV;
    }

    //printk(KERN_INFO "vlan_id = %d\n", vlan_id);
    if (vtss_if_mux_vlan_net_dev[vlan_id]) {
        printk(KERN_INFO "vlan exists already: %d\n", vlan_id);
        return -EEXIST;
    }

    parent_dev = vtss_if_mux_parent_dev_get();
    if (!parent_dev) {
        printk(KERN_INFO "No parent device\n");
        return -ENODEV;
    }

    //printk(KERN_INFO "reg new device\n");
    vtss_if_mux_dev_priv(dev)->vlan_id = vlan_id;
    vtss_if_mux_vlan_net_dev[vlan_id] = dev;
    err = register_netdevice(vtss_if_mux_vlan_net_dev[vlan_id]);
    if (err != 0) {
        printk(KERN_INFO "Failed to register device\n");
        goto exit_rx_unreg;
    }

    //netif_stacked_transfer_operstate(parent_dev, dev);
    netif_carrier_on(dev);

    printk(KERN_INFO "vtss_if_mux_newlink vlan=%u, addr=%p\n", vlan_id,
           vtss_if_mux_vlan_net_dev[vlan_id]);
    return 0;

exit_rx_unreg:
    rtnl_lock();
    netdev_rx_handler_unregister(parent_dev);
    rtnl_unlock();

    return err;
}

static size_t vtss_if_mux_get_size(const struct net_device *dev) {
    return nla_total_size(2) +    /* IFLA_VLAN_PROTOCOL */
           nla_total_size(2);     /* IFLA_VLAN_ID */
}

/*
static int vtss_if_mux_fill_info(struct sk_buff *skb,
                                 const struct net_device *dev) {
    //printk(KERN_INFO "vtss_if_mux_fill_info\n");
    return 0;
}
*/

static int vtss_if_mux_rt_notify_fill(struct sk_buff *skb,
                                      struct net_device *dev) {
    struct nlmsghdr *nlh;
    struct ndmsg *ndm;

    nlh = nlmsg_put(skb, 0, 0, RTM_GETNEIGH, sizeof(*ndm), 0);
    if (nlh == NULL)
        return -EMSGSIZE;

    ndm = nlmsg_data(nlh);
    ndm->ndm_family  = AF_UNSPEC;
    ndm->ndm_pad1    = 0;
    ndm->ndm_pad2    = 0;
    ndm->ndm_flags   = NTF_SELF;
    ndm->ndm_type    = RTM_GETNEIGH;
    ndm->ndm_ifindex = dev->ifindex;
    ndm->ndm_state   = 0;

    return nlmsg_end(skb, nlh);
}

void vtss_if_mux_rt_notify(struct net_device *dev) {
    int err;
    struct net *net = dev_net(dev);
    struct sk_buff *skb;

    skb = nlmsg_new(1024, GFP_KERNEL); // TODO
    if (skb == NULL) {
        printk(KERN_INFO "ALLOC ERROR\n");
        return;
    }

    err = vtss_if_mux_rt_notify_fill(skb, dev);

    if (err < 0) {
        printk(KERN_INFO "vtss_if_mux_rt_notify_fill ERROR\n");
        kfree_skb(skb);
        goto errout;
    }

    rtnl_notify(skb, net, 0, RTNLGRP_NEIGH, NULL, GFP_KERNEL);
    return;

errout:
    rtnl_set_sk_err(net, RTNLGRP_NOTIFY, err);
}

void vtss_if_mux_dellink(struct net_device *dev, struct list_head *head) {
    int i;

    printk(KERN_INFO "unregister_vlan_dev\n");
    unregister_netdevice_queue(dev, head);

    for (i = 0; i < VLAN_N_VID; ++i) {
        if (vtss_if_mux_vlan_net_dev[i] == dev) {
            vtss_if_mux_vlan_net_dev[i] = 0;
        }
    }
}

struct rtnl_link_ops vtss_if_mux_link_ops __read_mostly = {
    .kind         = "vtss_if_mux",
    .maxtype      = IFLA_VLAN_MAX,
    .policy       = vtss_if_mux_policy,
    .priv_size    = sizeof(struct vtss_if_mux_dev_priv),
    .setup        = vtss_if_mux_setup,
    .validate     = vtss_if_mux_validate,
    .newlink      = vtss_if_mux_newlink,
    .changelink   = vtss_if_mux_changelink,
    .dellink      = vtss_if_mux_dellink,
    .get_size     = vtss_if_mux_get_size,
    //.fill_info    = vtss_if_mux_fill_info,
};

int __init vtss_if_mux_netlink_init(void) {
    return rtnl_link_register(&vtss_if_mux_link_ops);
}

void __exit vtss_if_mux_netlink_uninit(void) {
    rtnl_link_unregister(&vtss_if_mux_link_ops);
}

//MODULE_ALIAS_RTNL_LINK("vtss_if_mux");
