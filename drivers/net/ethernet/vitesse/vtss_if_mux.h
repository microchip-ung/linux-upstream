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

#include <linux/if_vlan.h>
#include <linux/netdevice.h>
#include <linux/u64_stats_sync.h>

#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
#define IFH_ID   0x01  /* No IFH_ID in Luton26, madeup 0x01 */
#define IFH_LEN  8
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
#define IFH_ID   0x05
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

extern struct net_device *vtss_if_mux_parent_dev;
extern struct net_device *vtss_if_mux_vlan_net_dev[VLAN_N_VID];

void vtss_if_mux_setup(struct net_device *netdev);

int vtss_if_mux_netlink_init(void);
void vtss_if_mux_netlink_uninit(void);

rx_handler_result_t vtss_if_mux_rx_handler(struct sk_buff **pskb);
struct net_device *vtss_if_mux_parent_dev_get(void);

void vtss_if_mux_rt_notify(struct net_device *dev);

