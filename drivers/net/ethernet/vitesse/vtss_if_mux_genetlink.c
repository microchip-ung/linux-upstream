/*
 Vitesse Switch Software.

 Copyright (c) 2002-2015 Vitesse Semiconductor Corporation "Vitesse". All
 Rights Reserved.
*/

#include <net/ipv6.h>
#include <net/genetlink.h>
#include <uapi/linux/ipv6.h>
#include <linux/inetdevice.h>
#include "vtss_if_mux.h"

enum vtss_if_mux_action {
	VTSS_IF_MUX_ACTION_DROP,
	VTSS_IF_MUX_ACTION_CHECK_WHITE,
	VTSS_IF_MUX_ACTION_ACCEPT,
};

enum vtss_if_mux_list {
	VTSS_IF_MUX_LIST_WHITE,
	VTSS_IF_MUX_LIST_BLACK,
};

enum vtss_if_mux_attr {
	VTSS_IF_MUX_ATTR_NONE,
	VTSS_IF_MUX_ATTR_ID,
	VTSS_IF_MUX_ATTR_OWNER,
	VTSS_IF_MUX_ATTR_LIST,
	VTSS_IF_MUX_ATTR_ACTION,
	VTSS_IF_MUX_ATTR_RULE,
	VTSS_IF_MUX_ATTR_ELEMENT,
	VTSS_IF_MUX_ATTR_ELEMENT_TYPE,
	VTSS_IF_MUX_ATTR_ELEMENT_PORT_MASK,
	VTSS_IF_MUX_ATTR_ELEMENT_ADDR,
	VTSS_IF_MUX_ATTR_ELEMENT_INT,
	VTSS_IF_MUX_ATTR_ELEMENT_PREFIX,

	// Add new entries here, and remember to update user-space applications
	VTSS_IF_MUX_ATTR_END,
};
#define VTSS_IF_MUX_ATTR_MAX (VTSS_IF_MUX_ATTR_END - 1)

enum vtss_if_mux_genl {
	VTSS_IF_MUX_GENL_NOOP,
	VTSS_IF_MUX_GENL_RULE_CREATE,
	VTSS_IF_MUX_GENL_RULE_DELETE,
	VTSS_IF_MUX_GENL_RULE_MODIFY,
	VTSS_IF_MUX_GENL_RULE_GET,

	// Add new entries here, and remember to update user-space applications
};

enum vtss_if_mux_filter_type {
	VTSS_IF_MUX_FILTER_TYPE_none = 0,
	VTSS_IF_MUX_FILTER_TYPE_port_mask = 1,
	VTSS_IF_MUX_FILTER_TYPE_mac_src = 2,
	VTSS_IF_MUX_FILTER_TYPE_mac_dst = 3,
	VTSS_IF_MUX_FILTER_TYPE_mac_src_or_dst = 4,
	VTSS_IF_MUX_FILTER_TYPE_vlan = 5,
	VTSS_IF_MUX_FILTER_TYPE_ether_type = 6,
	VTSS_IF_MUX_FILTER_TYPE_ipv4_src = 7,
	VTSS_IF_MUX_FILTER_TYPE_ipv4_dst = 8,
	VTSS_IF_MUX_FILTER_TYPE_ipv4_src_or_dst = 9,
	VTSS_IF_MUX_FILTER_TYPE_ipv6_src = 10,
	VTSS_IF_MUX_FILTER_TYPE_ipv6_dst = 11,
	VTSS_IF_MUX_FILTER_TYPE_ipv6_src_or_dst = 12,
	VTSS_IF_MUX_FILTER_TYPE_arp_operation = 22,
	VTSS_IF_MUX_FILTER_TYPE_arp_hw_sender = 23,
	VTSS_IF_MUX_FILTER_TYPE_arp_hw_target = 24,
	VTSS_IF_MUX_FILTER_TYPE_arp_proto_sender = 25,
	VTSS_IF_MUX_FILTER_TYPE_arp_proto_target = 26,
};

static struct nla_policy genel_policy[VTSS_IF_MUX_ATTR_END] = {
		[VTSS_IF_MUX_ATTR_NONE] = {.type = NLA_UNSPEC},
		[VTSS_IF_MUX_ATTR_ID] = {.type = NLA_U32},
		[VTSS_IF_MUX_ATTR_OWNER] = {.type = NLA_U64},
		[VTSS_IF_MUX_ATTR_LIST] = {.type = NLA_U32},
		[VTSS_IF_MUX_ATTR_ACTION] = {.type = NLA_U32},
		[VTSS_IF_MUX_ATTR_RULE] = {.type = NLA_NESTED},
		[VTSS_IF_MUX_ATTR_ELEMENT] = {.type = NLA_NESTED},
		[VTSS_IF_MUX_ATTR_ELEMENT_TYPE] = {.type = NLA_U32},
		[VTSS_IF_MUX_ATTR_ELEMENT_PORT_MASK] = {.type = NLA_U64},
		[VTSS_IF_MUX_ATTR_ELEMENT_ADDR] = {.type = NLA_BINARY,
						   .len = MAX_ADDR_LEN},
		[VTSS_IF_MUX_ATTR_ELEMENT_INT] = {.type = NLA_U32},
};

struct vtss_if_mux_filter_element {
	// Type of element
	int type;

	// We need to hold the prefix and the ip addresses.
	int prefix;

	union {
		// Various numbers
		u32 i;

		// Used as port mask - must be converted to physical ports in
		// user-space
		u64 mask;

		// We need 16 bytes to hold a IPv6 address
		char address[16];
	} data;
};

struct vtss_if_mux_filter_rule {
	struct list_head list;
	struct rcu_head rcu;

	u32 id;
	u32 bitmaks_idx;

	enum vtss_if_mux_action action;

	u64 owner;

	int cnt;
	struct vtss_if_mux_filter_element elements[0];
};

static struct genl_family vtss_if_mux_genl_family = {
	.id = GENL_ID_GENERATE,
	.hdrsize = 0,
	.name = "vtss_if_mux",
	.version = 1,
	.maxattr = VTSS_IF_MUX_ATTR_MAX,
};

struct frame_data {
	unsigned int vid;
	struct sk_buff *skb;

	int fallback;

	unsigned int ether_type_offset;

	u64 whitelist_mask;
};

struct owner_bit_mask {
	struct list_head list;
	u32 bit_idx;
	u64 owner;
	u32 ref_cnt;
};

static DEFINE_MUTEX(vtss_if_mux_genl_sem);
static struct list_head VTSS_IF_MUX_FILTER_WHITE_LIST;
static struct list_head VTSS_IF_MUX_FILTER_BLACK_LIST;
#if defined(CONFIG_PROC_FS)
static struct proc_dir_entry *proc_dump = 0;
#endif
static u64 OWNER_BIT_MASK_POOL = 0;
static struct list_head OWNER_BIT_MASK_ASSOCIATION;

#define VTSS_HDR (IFH_LEN + 2)
#define ETHERTYPE_LENGTH 2
static inline int vtss_port_check(struct frame_data *d, u64 mask)
{
	u64 p = 0, m = 0;
#if defined(CONFIG_VTSS_VCOREIII_LUTON26)
	p = d->skb->data[3];
	p = (p >> 3);
	p &= 0x1f;

#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
	p = d->skb->data[12];
	p = (p >> 3);
	p &= 0xf;

#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
	p = d->skb->data[25] & 1;
	p <<= 5;
	p |= (d->skb->data[26] >> 3) & 0x1f;
	printk(KERN_ERR "CHIP-PORT: %llu - delete line when tested!", p);  // TODO

#else
#error Invalid architecture type
#endif
	m = 1llu << p;

	return (mask & m) != 0llu;
}

static inline int vtss_mac_src_check(struct frame_data *d, char *mac)
{
	return ether_addr_equal(mac, d->skb->data + VTSS_HDR + 6);
}

static inline int vtss_mac_dst_check(struct frame_data *d, char *mac)
{
	return ether_addr_equal(mac, d->skb->data + VTSS_HDR);
}

static inline int vtss_vlan_check(struct frame_data *d, u32 vid)
{
	return d->vid == vid;
}

static inline int vtss_ether_check(struct frame_data *d, u32 ether_type)
{
	__be16 *et = (u16 *)(d->skb->data + d->ether_type_offset);
	return ntohs(*et) == ether_type;
}

static inline struct iphdr *vtss_ipv4_hdr(struct frame_data *d)
{
	__be16 *et = (u16 *)(d->skb->data + d->ether_type_offset);
	if (*et != htons(ETH_P_IP))
		return NULL;

	if (d->skb->len <
	    (d->ether_type_offset + ETHERTYPE_LENGTH + sizeof(struct iphdr)))
		return NULL;

	return (struct iphdr *)(d->skb->data + d->ether_type_offset +
				ETHERTYPE_LENGTH);
}

static inline int vtss_ipv4_src_check(struct frame_data *d, char *addr, int p)
{
	struct iphdr *h;
	__be32 mask, match;

	h = vtss_ipv4_hdr(d);
	if (!h)
		return 0;

	match = *((__be32 *)addr);
	mask = inet_make_mask(p);

	return (h->saddr & mask) == (match & mask);
}

static inline int vtss_ipv4_dst_check(struct frame_data *d, char *addr, int p)
{
	struct iphdr *h;
	__be32 mask, match;

	h = vtss_ipv4_hdr(d);
	if (!h)
		return 0;

	match = *((__be32 *)addr);
	mask = inet_make_mask(p);

	return (h->daddr & mask) == (match & mask);
}

static inline struct ipv6hdr *vtss_ipv6_hdr(struct frame_data *d)
{
	__be16 *et = (__be16 *)(d->skb->data + d->ether_type_offset);
	if (*et != htons(ETH_P_IPV6))
		return NULL;

	if (d->skb->len <
	    (d->ether_type_offset + ETHERTYPE_LENGTH + sizeof(struct ipv6hdr)))
		return NULL;

	return (struct ipv6hdr *)(d->skb->data + d->ether_type_offset +
				  ETHERTYPE_LENGTH);
}

static inline int vtss_ipv6_src_check(struct frame_data *d, char *addr, int p)
{
	struct ipv6hdr *h;

	h = vtss_ipv6_hdr(d);
	if (!h)
		return 0;

	return ipv6_prefix_equal((struct in6_addr *)addr, &h->saddr, p);
}

static inline int vtss_ipv6_dst_check(struct frame_data *d, char *addr, int p)
{
	struct ipv6hdr *h;

	h = vtss_ipv6_hdr(d);
	if (!h)
		return 0;

	return ipv6_prefix_equal((struct in6_addr *)addr, &h->daddr, p);
}

static inline u8 *vtss_arp_hdr(struct frame_data *d)
{
	u16 *et = (u16 *)(d->skb->data + d->ether_type_offset);
	if (*et != htons(ETH_P_ARP))
		return NULL;

	if (d->skb->len < (d->ether_type_offset + ETHERTYPE_LENGTH + 28))
		return NULL;

	return (u8 *)d->skb->data + d->ether_type_offset + ETHERTYPE_LENGTH;
}

static inline int vtss_arp_operation_check(struct frame_data *d, int opr)
{
	__be16 *o;
	u8 *hdr = vtss_arp_hdr(d);

	if (!hdr)
		return 0;

	o = (__be16 *)(hdr + 6);

	return ntohs(*o) == opr;
}

static inline int vtss_arp_hw_sender_check(struct frame_data *d, char *addr)
{
	char *mac;
	char *hdr = (char *)vtss_arp_hdr(d);

	if (!hdr)
		return 0;

	mac = hdr + 8;

	return ether_addr_equal(addr, mac);
}

static inline int vtss_arp_hw_target_check(struct frame_data *d, char *addr)
{
	char *mac;
	char *hdr = (char *)vtss_arp_hdr(d);

	if (!hdr)
		return 0;

	mac = hdr + 18;

	return ether_addr_equal(addr, mac);
}

static inline int vtss_arp_proto_sender_check(struct frame_data *d, char *addr,
					      int p)
{
	__be32 *a, *b, mask;
	char *hdr = (char *)vtss_arp_hdr(d);

	if (!hdr)
		return 0;

	a = (__be32 *)addr;
	b = (__be32 *)(hdr + 14);
	mask = inet_make_mask(p);

	return (*a & mask) == (*b & mask);
}

static inline int vtss_arp_proto_target_check(struct frame_data *d, char *addr,
					      int p)
{
	__be32 *a, *b, mask;
	char *hdr = (char *)vtss_arp_hdr(d);

	if (!hdr)
		return 0;

	a = (__be32 *)addr;
	b = (__be32 *)(hdr + 24);
	mask = inet_make_mask(p);

	return (*a & mask) == (*b & mask);
}

static inline int element_match(struct vtss_if_mux_filter_element *e,
				struct frame_data *d)
{
	switch (e->type) {
	case VTSS_IF_MUX_FILTER_TYPE_port_mask:
		return vtss_port_check(d, e->data.mask);

	case VTSS_IF_MUX_FILTER_TYPE_mac_src:
		return vtss_mac_src_check(d, e->data.address);

	case VTSS_IF_MUX_FILTER_TYPE_mac_dst:
		return vtss_mac_dst_check(d, e->data.address);

	case VTSS_IF_MUX_FILTER_TYPE_mac_src_or_dst:
		return vtss_mac_src_check(d, e->data.address) ||
		       vtss_mac_dst_check(d, e->data.address);

	case VTSS_IF_MUX_FILTER_TYPE_vlan:
		return vtss_vlan_check(d, e->data.i);

	case VTSS_IF_MUX_FILTER_TYPE_ether_type:
		return vtss_ether_check(d, e->data.i);

	case VTSS_IF_MUX_FILTER_TYPE_ipv4_src:
		return vtss_ipv4_src_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_ipv4_dst:
		return vtss_ipv4_dst_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_ipv4_src_or_dst:
		return vtss_ipv4_src_check(d, e->data.address, e->prefix) ||
		       vtss_ipv4_dst_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_ipv6_src:
		return vtss_ipv6_src_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_ipv6_dst:
		return vtss_ipv6_dst_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_ipv6_src_or_dst:
		return vtss_ipv6_src_check(d, e->data.address, e->prefix) ||
		       vtss_ipv6_dst_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_arp_operation:
		return vtss_arp_operation_check(d, e->data.i);

	case VTSS_IF_MUX_FILTER_TYPE_arp_hw_sender:
		return vtss_arp_hw_sender_check(d, e->data.address);

	case VTSS_IF_MUX_FILTER_TYPE_arp_hw_target:
		return vtss_arp_hw_target_check(d, e->data.address);

	case VTSS_IF_MUX_FILTER_TYPE_arp_proto_sender:
		return vtss_arp_proto_sender_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_arp_proto_target:
		return vtss_arp_proto_target_check(d, e->data.address, e->prefix);

	case VTSS_IF_MUX_FILTER_TYPE_none:
	default:
		return d->fallback;
	}

	return d->fallback;
}

static inline int rule_match(struct vtss_if_mux_filter_rule *r,
			     struct frame_data *d)
{
	int i;

	for (i = 0; i < r->cnt; ++i) {
		if (!element_match(&r->elements[i], d)) {
			// printk(KERN_ERR "Element %d did not match\n", i);
			return 0;
		}
	}

	// printk(KERN_ERR "All elements match\n");
	return 1;
}

static inline enum vtss_if_mux_action apply_black_list(struct frame_data *d)
{
	struct vtss_if_mux_filter_rule *r;
	enum vtss_if_mux_action a = VTSS_IF_MUX_ACTION_ACCEPT;
	d->fallback = 1;
	d->whitelist_mask = 0;

	// printk(KERN_ERR "Black list\n");
	rcu_read_lock();
	list_for_each_entry_rcu (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		if (rule_match(r, d)) {
			a = r->action;
			if (a == VTSS_IF_MUX_ACTION_DROP) {
				break;

			} else if (a == VTSS_IF_MUX_ACTION_CHECK_WHITE) {
				u64 m = 1;
				m <<= r->bitmaks_idx;
				d->whitelist_mask &= m;

			} else {
				BUG();
			}
		}
	}
	rcu_read_unlock();

	// printk(KERN_ERR "Black list -> %d\n", a);
	return a;
}

static inline enum vtss_if_mux_action apply_white_list(struct frame_data *d)
{
	struct vtss_if_mux_filter_rule *r;
	d->fallback = 0;

	// printk(KERN_ERR "White list\n");
	rcu_read_lock();
	list_for_each_entry_rcu (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		u64 m = 1;
		m <<= r->bitmaks_idx;

		if ((d->whitelist_mask & m) == 0llu)
			continue;

		if (rule_match(r, d)) {
			d->whitelist_mask &= ~m;
		}

		if (!d->whitelist_mask)
			break;
	}
	rcu_read_unlock();

	if (d->whitelist_mask) {
		// printk(KERN_ERR "White list drop 0x%08llx\n",
		// d->whitelist_mask);
		return VTSS_IF_MUX_ACTION_DROP;
	} else {
		// printk(KERN_ERR "White accept\n");
		return VTSS_IF_MUX_ACTION_ACCEPT;
	}

	// Unreachable
	return VTSS_IF_MUX_ACTION_DROP;
}

int vtss_if_mux_filter_apply(struct sk_buff *skb, unsigned int vid,
			     unsigned int ether_type_offset)
{
	int res = 1;
	enum vtss_if_mux_action black_action;
	struct frame_data d = {};
	d.vid = vid;
	d.skb = skb;
	d.ether_type_offset = ether_type_offset;

	// printk(KERN_ERR "%d\n", __LINE__);
	// print_hex_dump(KERN_ERR, "RX: ", DUMP_PREFIX_OFFSET, 16, 1,
	//                skb->data, skb->len, false);
	black_action = apply_black_list(&d);
	switch (black_action) {
	case VTSS_IF_MUX_ACTION_DROP:
		res = 1;
		break;

	case VTSS_IF_MUX_ACTION_CHECK_WHITE:
		res = 1;
		break;

	case VTSS_IF_MUX_ACTION_ACCEPT:
		res = 0;
		break;

	default:
		res = 1;
		printk(KERN_ERR "ERROR!! %d\n", __LINE__);
		break;
	}

	if (black_action == VTSS_IF_MUX_ACTION_CHECK_WHITE) {
		black_action = apply_white_list(&d);
		switch (black_action) {
		case VTSS_IF_MUX_ACTION_DROP:
			res = 1;
			break;

		case VTSS_IF_MUX_ACTION_ACCEPT:
			res = 0;
			break;

		default:
			res = 1;
			printk(KERN_ERR "ERROR!! %d\n", __LINE__);
			break;
		}
	}

	return res;
}

static int filter_size(int element_cnt)
{
	int alloc_size = sizeof(struct vtss_if_mux_filter_rule);
	alloc_size += element_cnt * sizeof(struct vtss_if_mux_filter_element);
	return alloc_size;
}

static int get_free_id(void)
{
	static u32 last_id = 0;
	struct vtss_if_mux_filter_rule *r;

	mutex_lock(&vtss_if_mux_genl_sem);
	rcu_read_lock();
AGAIN:
	last_id++;

	// Handle wrap around
	if (last_id <= 0) {
		last_id = 0;
		goto AGAIN;
	}

	list_for_each_entry_rcu (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		if (r->id == last_id)
			goto AGAIN;
	}

	list_for_each_entry_rcu (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		if (r->id == last_id)
			goto AGAIN;
	}

	rcu_read_unlock();
	mutex_unlock(&vtss_if_mux_genl_sem);
	return last_id;
};

static int vtss_if_mux_genl_cmd_noop(struct sk_buff *skb, struct genl_info *info)
{
	return 0;
}

static int bitmask_idx_alloc(struct vtss_if_mux_filter_rule *r)
{
	// TODO, assume locked
	u32 i;
	u64 mask;
	int bit_idx = -1;
	struct owner_bit_mask *e;

	// Check the cache
	list_for_each_entry (e, &OWNER_BIT_MASK_ASSOCIATION, list) {
		if (e->owner == r->owner) {
			e->ref_cnt++;
			r->bitmaks_idx = e->bit_idx;
			return 0;
		}
	}

	// Find an non-used bit
	mask = 1;
	for (i = 0; i < sizeof(OWNER_BIT_MASK_POOL) * 8; ++i) {
		if ((OWNER_BIT_MASK_POOL & mask) == 0llu) {
			bit_idx = i;
			break;
		}

		mask <<= 1;
	}

	if (bit_idx < 0) {
		printk(KERN_ERR "Too many owners\n");
		return -1;  // TODO, find better error value
	}

	e = kmalloc(sizeof(struct owner_bit_mask), GFP_KERNEL | __GFP_ZERO);
	if (!e)
		return -ENOMEM;

	e->bit_idx = bit_idx;
	e->owner = r->owner;
	e->ref_cnt = 1;
	OWNER_BIT_MASK_POOL &= mask;
	list_add(&e->list, &OWNER_BIT_MASK_ASSOCIATION);

	return 0;
}

static void bitmask_idx_free(struct vtss_if_mux_filter_rule *r)
{
	// TODO, assume locked
	u64 mask;
	int bit_idx = -1;
	struct owner_bit_mask *e;

	// Check the cache
	list_for_each_entry (e, &OWNER_BIT_MASK_ASSOCIATION, list) {
		if (e->owner == r->owner) {
			if (e->ref_cnt > 1) {
				e->ref_cnt -= 1;
				return;
			} else {
				bit_idx = e->bit_idx;
				break;
			}
		}
	}

	BUG_ON(bit_idx == -1);

	mask = 1;
	mask <<= bit_idx;
	OWNER_BIT_MASK_POOL &= ~mask;

	list_del(&e->list);
	kfree(e);
}

void rule_free(struct rcu_head *head)
{
	struct vtss_if_mux_filter_rule *r =
		container_of(head, struct vtss_if_mux_filter_rule, rcu);

	mutex_lock(&vtss_if_mux_genl_sem);
	bitmask_idx_free(r);
	mutex_unlock(&vtss_if_mux_genl_sem);

	kfree(r);
}

static int parse_elements(struct vtss_if_mux_filter_element *e,
			  struct nlattr *rule)
{
	int err;
	struct nlattr *element_attr[VTSS_IF_MUX_ATTR_END];

	err = nla_parse_nested(element_attr, VTSS_IF_MUX_ATTR_MAX, rule,
			       genel_policy);
	if (err < 0) {
		printk(KERN_ERR "Failed to parse rule-elements\n");
		return -EINVAL;
	}

	if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_TYPE]) {
		printk(KERN_ERR "No type!\n");
		return -EINVAL;
	}

	e->type = nla_get_u32(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_TYPE]);

	// The element data is requiret, but depends on the type
	switch (e->type) {
	case VTSS_IF_MUX_FILTER_TYPE_port_mask:
		if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_PORT_MASK]) {
			printk(KERN_ERR "No port mask!\n");
			return -EINVAL;
		}
		e->data.mask = nla_get_u64(
			element_attr[VTSS_IF_MUX_ATTR_ELEMENT_PORT_MASK]);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_mac_src:
	case VTSS_IF_MUX_FILTER_TYPE_mac_dst:
	case VTSS_IF_MUX_FILTER_TYPE_mac_src_or_dst:
	case VTSS_IF_MUX_FILTER_TYPE_arp_hw_sender:
	case VTSS_IF_MUX_FILTER_TYPE_arp_hw_target:
		if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]) {
			printk(KERN_ERR "No address!\n");
			return -EINVAL;
		}

		if (nla_len(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]) != 6) {
			printk(KERN_ERR "Unexpected length: %d!\n",
			       nla_len(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]));
			return -EINVAL;
		}

		memcpy(e->data.address,
		       nla_data(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]), 6);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_vlan:
	case VTSS_IF_MUX_FILTER_TYPE_ether_type:
	case VTSS_IF_MUX_FILTER_TYPE_arp_operation:
		if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_INT]) {
			printk(KERN_ERR "No int!\n");
			return -EINVAL;
		}
		e->data.mask =
			nla_get_u32(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_INT]);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_ipv4_src:
	case VTSS_IF_MUX_FILTER_TYPE_ipv4_dst:
	case VTSS_IF_MUX_FILTER_TYPE_ipv4_src_or_dst:
	case VTSS_IF_MUX_FILTER_TYPE_arp_proto_sender:
	case VTSS_IF_MUX_FILTER_TYPE_arp_proto_target:
		if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]) {
			printk(KERN_ERR "No address!\n");
			return -EINVAL;
		}

		if (nla_len(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]) != 4) {
			printk(KERN_ERR "Unexpected length: %d!\n",
			       nla_len(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]));
			return -EINVAL;
		}

		memcpy(e->data.address,
		       nla_data(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]), 4);

		if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_PREFIX]) {
			printk(KERN_ERR "No prefix!\n");
			return -EINVAL;
		}

		e->prefix = nla_get_u32(
			element_attr[VTSS_IF_MUX_ATTR_ELEMENT_PREFIX]);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_ipv6_src:
	case VTSS_IF_MUX_FILTER_TYPE_ipv6_dst:
	case VTSS_IF_MUX_FILTER_TYPE_ipv6_src_or_dst:
		if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]) {
			printk(KERN_ERR "No address!\n");
			return -EINVAL;
		}

		if (nla_len(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]) != 16) {
			printk(KERN_ERR "Unexpected length: %d!\n",
			       nla_len(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]));
			return -EINVAL;
		}

		memcpy(e->data.address,
		       nla_data(element_attr[VTSS_IF_MUX_ATTR_ELEMENT_ADDR]), 16);

		if (!element_attr[VTSS_IF_MUX_ATTR_ELEMENT_PREFIX]) {
			printk(KERN_ERR "No prefix!\n");
			return -EINVAL;
		}

		e->prefix = nla_get_u32(
			element_attr[VTSS_IF_MUX_ATTR_ELEMENT_PREFIX]);
		break;

	default:
		printk(KERN_ERR "Unsupported type!\n");
		return -EINVAL;
	}

	return 0;
}

static struct vtss_if_mux_filter_rule *parse_rule(struct sk_buff *skb,
						  struct genl_info *info,
						  int id, int *err)
{
	struct vtss_if_mux_filter_rule *r = NULL;
	struct nlattr *rule;
	int element_cnt = 0;
	int rem, i;

	// Not sure if there is a better way to get the number of elements
	if (info->attrs[VTSS_IF_MUX_ATTR_RULE]) {
		nla_for_each_nested (rule, info->attrs[VTSS_IF_MUX_ATTR_RULE],
				     rem) {
			element_cnt += 1;
		}
	}

	r = kmalloc(filter_size(element_cnt), GFP_KERNEL | __GFP_ZERO);
	if (!r) {
		*err = -ENOMEM;
		return NULL;
	}

	r->id = id;

	if (info->attrs[VTSS_IF_MUX_ATTR_OWNER]) {
		r->owner = nla_get_u64(info->attrs[VTSS_IF_MUX_ATTR_OWNER]);
	} else {
		*err = -ENOMEM;
		goto ERR;
	}

	// Validate and copy the desiered action.
	if (info->attrs[VTSS_IF_MUX_ATTR_ACTION]) {
		switch (nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_ACTION])) {
		case VTSS_IF_MUX_ACTION_DROP:
			r->action = VTSS_IF_MUX_ACTION_DROP;
			break;

		case VTSS_IF_MUX_ACTION_CHECK_WHITE:
			r->action = VTSS_IF_MUX_ACTION_CHECK_WHITE;
			break;

		default:
			*err = -EINVAL;
			goto ERR;
		}
	} else {
		r->action = VTSS_IF_MUX_ACTION_DROP;
	}

	// Copy all the elements from the netlink message into the allocated
	// rule
	i = 0;
	nla_for_each_nested (rule, info->attrs[VTSS_IF_MUX_ATTR_RULE], rem) {
		BUG_ON(i >= element_cnt);

		if (parse_elements(&(r->elements[i]), rule) < 0) {
			*err = -EINVAL;
			goto ERR;
		}

		// Rule has been added, increment the count.
		r->cnt++;
		i++;
	}

	return r;

ERR:
	kfree(r);
	return NULL;
}

static int vtss_if_mux_genl_cmd_rule_create(struct sk_buff *skb,
					    struct genl_info *info)
{
	int id;
	void *hdr;
	int err = -1;
	struct vtss_if_mux_filter_rule *r = NULL;
	struct sk_buff *msg;

	if (!info->attrs[VTSS_IF_MUX_ATTR_LIST])
		return -EINVAL;

	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq,
			  &vtss_if_mux_genl_family, 0,
			  VTSS_IF_MUX_GENL_RULE_CREATE);
	if (!hdr) {
		err = -EMSGSIZE;
		goto ERROR_MEM_MSG;
	}

	id = get_free_id();
	if (nla_put_u32(msg, VTSS_IF_MUX_ATTR_ID, id)) {
		err = -EMSGSIZE;
		goto ERROR_GENLMSG;
	}

	r = parse_rule(skb, info, id, &err);
	if (!r)
		goto ERROR_GENLMSG;

	err = bitmask_idx_alloc(r);
	if (err < 0)
		goto ERROR_MEM_RULE;

	// Install the rule into the configured list
	switch (nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_LIST])) {
	case VTSS_IF_MUX_LIST_WHITE:
		mutex_lock(&vtss_if_mux_genl_sem);
		list_add_rcu(&r->list, &VTSS_IF_MUX_FILTER_WHITE_LIST);
		mutex_unlock(&vtss_if_mux_genl_sem);
		break;

	case VTSS_IF_MUX_LIST_BLACK:
		mutex_lock(&vtss_if_mux_genl_sem);
		list_add_rcu(&r->list, &VTSS_IF_MUX_FILTER_BLACK_LIST);
		mutex_unlock(&vtss_if_mux_genl_sem);
		break;

	default:
		err = -EINVAL;
		goto ERROR_BITMASK;
	}

	genlmsg_end(msg, hdr);
	return genlmsg_unicast(genl_info_net(info), msg, info->snd_portid);

ERROR_BITMASK:
	bitmask_idx_free(r);

ERROR_MEM_RULE:
	kfree(r);

ERROR_GENLMSG:
	genlmsg_cancel(skb, hdr);

ERROR_MEM_MSG:
	nlmsg_free(msg);

	return err;
}

struct vtss_if_mux_filter_rule *find_rule_by_id(int id,
						enum vtss_if_mux_list *list)
{
	struct vtss_if_mux_filter_rule *res = NULL, *r;

	rcu_read_lock();
	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		if (r->id == id) {
			BUG_ON(res);
			res = r;
			if (list)
				*list = VTSS_IF_MUX_LIST_BLACK;
		}
	}

	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		if (r->id == id) {
			BUG_ON(res);
			res = r;
			if (list)
				*list = VTSS_IF_MUX_LIST_BLACK;
		}
	}
	rcu_read_unlock();

	return res;
}

static int genl_rule_del_by_id(u32 id)
{
	struct vtss_if_mux_filter_rule *r;

	r = find_rule_by_id(id, NULL);

	if (!r)
		return -ENOENT;

	mutex_lock(&vtss_if_mux_genl_sem);
	list_del_rcu(&r->list);
	call_rcu(&r->rcu, rule_free);
	mutex_unlock(&vtss_if_mux_genl_sem);

	return 0;
}

// TODO, should be owner,pid
static int genl_rule_del_by_owner(u64 owner)
{
	int cnt = 0;
	struct vtss_if_mux_filter_rule *r;

	mutex_lock(&vtss_if_mux_genl_sem);
	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		if (r->owner == owner) {
			cnt++;
			list_del_rcu(&r->list);
			call_rcu(&r->rcu, rule_free);
		}
	}

	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		if (r->owner == owner) {
			cnt++;
			list_del_rcu(&r->list);
			call_rcu(&r->rcu, rule_free);
		}
	}
	mutex_unlock(&vtss_if_mux_genl_sem);

	return cnt;
}

static int genl_rule_del_all(void)
{
	int cnt = 0;
	struct vtss_if_mux_filter_rule *r;

	mutex_lock(&vtss_if_mux_genl_sem);
	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		cnt++;
		list_del_rcu(&r->list);
		call_rcu(&r->rcu, rule_free);
	}

	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		cnt++;
		list_del_rcu(&r->list);
		call_rcu(&r->rcu, rule_free);
	}
	mutex_unlock(&vtss_if_mux_genl_sem);

	return cnt;
}

static int genl_cmd_rule_delte(struct sk_buff *skb, struct genl_info *info)
{
	if (info->attrs[VTSS_IF_MUX_ATTR_ID] &&
	    info->attrs[VTSS_IF_MUX_ATTR_OWNER])
		return -EINVAL;

	if (info->attrs[VTSS_IF_MUX_ATTR_ID]) {
		return genl_rule_del_by_id(
			nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_ID]));

	} else if (info->attrs[VTSS_IF_MUX_ATTR_OWNER]) {
		genl_rule_del_by_owner(
			nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_ID]));
		return 0;

	} else {
		genl_rule_del_all();
		return 0;
	}

	return -EINVAL;
}

static int genl_cmd_rule_modify(struct sk_buff *skb, struct genl_info *info)
{
	int id;
	int err = -1;
	enum vtss_if_mux_list list_old;
	struct vtss_if_mux_filter_rule *r_new = NULL, *r_old = NULL;

	if (!info->attrs[VTSS_IF_MUX_ATTR_ID])
		return -EINVAL;

	id = nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_ID]);

	r_new = parse_rule(skb, info, id, &err);
	if (!r_new)
		goto ERROR;

	r_old = find_rule_by_id(id, &list_old);
	if (!r_old) {
		err = -ENOENT;
		goto ERROR;
	}

	if (!info->attrs[VTSS_IF_MUX_ATTR_OWNER])
		r_new->owner = r_old->owner;

	if (!info->attrs[VTSS_IF_MUX_ATTR_ACTION])
		r_new->action = r_old->action;

	err = bitmask_idx_alloc(r_new);
	if (err < 0)
		goto ERROR;

	// If a list is provided, then ensure that it is the same list as the
	// rule was found in. It is not allowed to move a rule from one list to
	// another by doing an update.
	if (info->attrs[VTSS_IF_MUX_ATTR_LIST]) {
		switch (nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_LIST])) {
		case VTSS_IF_MUX_LIST_WHITE:
			if (list_old != VTSS_IF_MUX_LIST_WHITE) {
				err = -EINVAL;
				goto ERROR_BITMASK;
			}
			break;

		case VTSS_IF_MUX_LIST_BLACK:
			if (list_old != VTSS_IF_MUX_LIST_WHITE) {
				err = -EINVAL;
				goto ERROR_BITMASK;
			}
			break;

		default:
			err = -EINVAL;
			goto ERROR_BITMASK;
		}
	}

	mutex_lock(&vtss_if_mux_genl_sem);
	list_replace_rcu(&r_old->list, &r_new->list);
	call_rcu(&r_old->rcu, rule_free);
	mutex_unlock(&vtss_if_mux_genl_sem);

	return 0;

ERROR_BITMASK:
	bitmask_idx_free(r_new);

ERROR:
	kfree(r_new);
	return err;
}

static int genl_cmd_rule_get(struct sk_buff *skb, struct genl_info *info)
{
	printk(KERN_ERR "Not implemented yet: genl_cmd_rule_get\n");
	return -ENOSYS;
}

static int genl_cmd_rule_dump(struct sk_buff *skb, struct netlink_callback *cb)
{
	printk(KERN_ERR "Not implemented yet: genl_cmd_rule_dump\n");
	return -ENOSYS;
}

void debug_dump_element(struct seq_file *s, struct vtss_if_mux_filter_element *e)
{
#define CASE(X)                                \
	case VTSS_IF_MUX_FILTER_TYPE_##X:      \
		seq_printf(s, "    " #X ": "); \
		break

	switch (e->type) {
		CASE(port_mask);
		CASE(mac_src);
		CASE(mac_dst);
		CASE(mac_src_or_dst);
		CASE(vlan);
		CASE(ether_type);
		CASE(ipv4_src);
		CASE(ipv4_dst);
		CASE(ipv4_src_or_dst);
		CASE(ipv6_src);
		CASE(ipv6_dst);
		CASE(ipv6_src_or_dst);
		CASE(arp_operation);
		CASE(arp_hw_sender);
		CASE(arp_hw_target);
		CASE(arp_proto_sender);
		CASE(arp_proto_target);
	default:
		seq_printf(s, "    UNKNOWN!!!\n");
		return;
	}
#undef CASE

	switch (e->type) {
	case VTSS_IF_MUX_FILTER_TYPE_port_mask:
		seq_printf(s, "0x%08llx", e->data.mask);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_mac_src:
	case VTSS_IF_MUX_FILTER_TYPE_mac_dst:
	case VTSS_IF_MUX_FILTER_TYPE_mac_src_or_dst:
	case VTSS_IF_MUX_FILTER_TYPE_arp_hw_sender:
	case VTSS_IF_MUX_FILTER_TYPE_arp_hw_target:
		seq_printf(s, "%02x:%02x:%02x:%02x:%02x:%02x",
			   e->data.address[0], e->data.address[1],
			   e->data.address[2], e->data.address[3],
			   e->data.address[4], e->data.address[5]);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_vlan:
		seq_printf(s, "%u", e->data.i);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_ether_type:
	case VTSS_IF_MUX_FILTER_TYPE_arp_operation:
		seq_printf(s, "0x%04x", e->data.i);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_ipv4_src:
	case VTSS_IF_MUX_FILTER_TYPE_ipv4_dst:
	case VTSS_IF_MUX_FILTER_TYPE_ipv4_src_or_dst:
	case VTSS_IF_MUX_FILTER_TYPE_arp_proto_sender:
	case VTSS_IF_MUX_FILTER_TYPE_arp_proto_target:
		seq_printf(s, "%hhu.%hhu.%hhu.%hhu/%d", e->data.address[0],
			   e->data.address[1], e->data.address[2],
			   e->data.address[3], e->prefix);
		break;

	case VTSS_IF_MUX_FILTER_TYPE_ipv6_src:
	case VTSS_IF_MUX_FILTER_TYPE_ipv6_dst:
	case VTSS_IF_MUX_FILTER_TYPE_ipv6_src_or_dst:
		seq_printf(s,
			   "%02x%02x:%02x%02x:%02x%02x:%02x%02x:"
			   "%02x%02x:%02x%02x:%02x%02x:%02x%02x/%d",
			   e->data.address[0], e->data.address[1],
			   e->data.address[2], e->data.address[3],
			   e->data.address[4], e->data.address[5],
			   e->data.address[6], e->data.address[7],
			   e->data.address[8], e->data.address[9],
			   e->data.address[10], e->data.address[11],
			   e->data.address[12], e->data.address[13],
			   e->data.address[14], e->data.address[15], e->prefix);
		break;
	}

	seq_printf(s, "\n");
}

static void debug_dump_rule(struct seq_file *s,
			    struct vtss_if_mux_filter_rule *r, bool black)
{
	int i;
	seq_printf(s, "  ID: %d, Owner: 0x%08llx", r->id, r->owner);
	if (black) {
		switch (r->action) {
		case VTSS_IF_MUX_ACTION_DROP:
			seq_printf(s, ", Action: DROP");
			break;
		case VTSS_IF_MUX_ACTION_CHECK_WHITE:
			seq_printf(s, ", Action: CHECK-WHITE-LIST");
			break;
		default:
			seq_printf(s, ", Action: UNKNOWN");
		}
	} else {
	}
	seq_printf(s, ", #Elements: %d\n", r->cnt);

	for (i = 0; i < r->cnt; ++i) debug_dump_element(s, &r->elements[i]);
}

static int debug_dump_(struct seq_file *s, void *v)
{
	struct vtss_if_mux_filter_rule *r;

	seq_printf(s, "BLACK_LIST:\n");
	rcu_read_lock();
	list_for_each_entry_rcu (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		debug_dump_rule(s, r, true);
	}

	seq_printf(s, "WHITE_LIST:\n");
	list_for_each_entry_rcu (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		debug_dump_rule(s, r, false);
	}
	rcu_read_unlock();
	seq_printf(s, "END\n");

	return 0;
}

static int debug_dump(struct inode *inode, struct file *f)
{
	return single_open(f, debug_dump_, NULL);
}

static struct genl_ops vtss_if_mux_genl_ops[] = {
	{
	 .cmd = VTSS_IF_MUX_GENL_NOOP,
	 .doit = vtss_if_mux_genl_cmd_noop,
	 .policy = genel_policy,
	 // No access control
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_CREATE,
	 .doit = vtss_if_mux_genl_cmd_rule_create,
	 .policy = genel_policy,
	 .flags = GENL_ADMIN_PERM,
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_DELETE,
	 .doit = genl_cmd_rule_delte,
	 .policy = genel_policy,
	 .flags = GENL_ADMIN_PERM,
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_MODIFY,
	 .doit = genl_cmd_rule_modify,
	 .policy = genel_policy,
	 .flags = GENL_ADMIN_PERM,
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_GET,
	 .doit = genl_cmd_rule_get,
	 .dumpit = genl_cmd_rule_dump,
	 .policy = genel_policy,
	 .flags = GENL_ADMIN_PERM,
	},
};

static const struct file_operations dump_fops = {
	.open = debug_dump,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int vtss_if_mux_genetlink_init(void)
{
	int err;

	INIT_LIST_HEAD_RCU(&VTSS_IF_MUX_FILTER_WHITE_LIST);
	INIT_LIST_HEAD_RCU(&VTSS_IF_MUX_FILTER_BLACK_LIST);
	INIT_LIST_HEAD(&OWNER_BIT_MASK_ASSOCIATION);

#if defined(CONFIG_PROC_FS)
	proc_dump = proc_create("vtss_if_mux_filter", S_IRUGO, NULL, &dump_fops);
#endif

	err = genl_register_family_with_ops(&vtss_if_mux_genl_family,
					    vtss_if_mux_genl_ops);
	if (err == -1) {
		printk(KERN_ERR "genl_register_family_with_ops failed\n");
	}

	return err;
}

void vtss_if_mux_genetlink_uninit(void)
{
#if defined(CONFIG_PROC_FS)
	if (proc_dump)
		proc_remove(proc_dump);
#endif

	genl_unregister_family(&vtss_if_mux_genl_family);
}
