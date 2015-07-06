/*
 Vitesse Switch Software.

 Copyright (c) 2002-2015 Vitesse Semiconductor Corporation "Vitesse". All
 Rights Reserved.
*/

#include <net/genetlink.h>
#include "vtss_if_mux.h"

enum vtss_if_mux_action {
	VTSS_IF_MUX_ACTION_DROP,
	VTSS_IF_MUX_ACTION_CHECK_WHITE,
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

static struct nla_policy vtss_if_mux_genel_policy[VTSS_IF_MUX_ATTR_END] = {
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

static DEFINE_MUTEX(vtss_if_mux_genl_sem);
static struct list_head VTSS_IF_MUX_FILTER_WHITE_LIST;
static struct list_head VTSS_IF_MUX_FILTER_BLACK_LIST;
#if defined(CONFIG_PROC_FS)
static struct proc_dir_entry *proc_dump = 0;
#endif

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

static int parse_elements(struct vtss_if_mux_filter_element *e,
			  struct nlattr *rule)
{
	int err;
	struct nlattr *element_attr[VTSS_IF_MUX_ATTR_END];

	err = nla_parse_nested(element_attr, VTSS_IF_MUX_ATTR_MAX, rule,
			       vtss_if_mux_genel_policy);
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

static struct vtss_if_mux_filter_rule *
parse_rule(struct sk_buff *skb, struct genl_info *info, int id, int *err)
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
	return 0;
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
		goto ERROR_MEM_RULE;
	}

	genlmsg_end(msg, hdr);
	return genlmsg_unicast(genl_info_net(info), msg, info->snd_portid);

ERROR_MEM_RULE:
	kfree(r);

ERROR_GENLMSG:
	genlmsg_cancel(skb, hdr);

ERROR_MEM_MSG:
	nlmsg_free(msg);

	return err;
}

struct vtss_if_mux_filter_rule *
vtss_if_mux_find_by_id(int id, enum vtss_if_mux_list *list)
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

static int vtss_if_mux_genl_rule_del_by_id(u32 id)
{
	struct vtss_if_mux_filter_rule *r;

	r = vtss_if_mux_find_by_id(id, NULL);

	if (!r)
		return -ENOENT;

	mutex_lock(&vtss_if_mux_genl_sem);
	list_del_rcu(&r->list);
	mutex_unlock(&vtss_if_mux_genl_sem);

	return 0;
}

// TODO, should be owner,pid
static int vtss_if_mux_genl_rule_del_by_owner(u64 owner)
{
	int cnt = 0;
	struct vtss_if_mux_filter_rule *r;

	mutex_lock(&vtss_if_mux_genl_sem);
	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		if (r->owner == owner) {
			cnt++;
			list_del_rcu(&r->list);
			kfree_rcu(r, rcu);
		}
	}

	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		if (r->owner == owner) {
			cnt++;
			list_del_rcu(&r->list);
			kfree_rcu(r, rcu);
		}
	}
	mutex_unlock(&vtss_if_mux_genl_sem);

	return cnt;
}

static int vtss_if_mux_genl_rule_del_all(void)
{
	int cnt = 0;
	struct vtss_if_mux_filter_rule *r;

	mutex_lock(&vtss_if_mux_genl_sem);
	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_BLACK_LIST, list) {
		cnt++;
		list_del_rcu(&r->list);
		kfree_rcu(r, rcu);
	}

	list_for_each_entry (r, &VTSS_IF_MUX_FILTER_WHITE_LIST, list) {
		cnt++;
		list_del_rcu(&r->list);
		kfree_rcu(r, rcu);
	}
	mutex_unlock(&vtss_if_mux_genl_sem);

	return cnt;
}

static int vtss_if_mux_genl_cmd_rule_delte(struct sk_buff *skb,
					   struct genl_info *info)
{
	if (info->attrs[VTSS_IF_MUX_ATTR_ID] &&
	    info->attrs[VTSS_IF_MUX_ATTR_OWNER])
		return -EINVAL;

	if (info->attrs[VTSS_IF_MUX_ATTR_ID]) {
		return vtss_if_mux_genl_rule_del_by_id(
			nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_ID]));

	} else if (info->attrs[VTSS_IF_MUX_ATTR_OWNER]) {
		vtss_if_mux_genl_rule_del_by_owner(
			nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_ID]));
		return 0;

	} else {
		vtss_if_mux_genl_rule_del_all();
		return 0;
	}

	return -EINVAL;
}

static int vtss_if_mux_genl_cmd_rule_modify(struct sk_buff *skb,
					    struct genl_info *info)
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

	r_old = vtss_if_mux_find_by_id(id, &list_old);
	if (!r_old) {
		err = -ENOENT;
		goto ERROR;
	}

	if (!info->attrs[VTSS_IF_MUX_ATTR_OWNER])
		r_new->owner = r_old->owner;

	if (!info->attrs[VTSS_IF_MUX_ATTR_ACTION])
		r_new->action = r_old->action;

	// If a list is provided, then ensure that it is the same list as the
	// rule was found in. It is not allowed to move a rule from one list to
	// another by doing an update.
	if (info->attrs[VTSS_IF_MUX_ATTR_LIST]) {
		switch (nla_get_u32(info->attrs[VTSS_IF_MUX_ATTR_LIST])) {
		case VTSS_IF_MUX_LIST_WHITE:
			if (list_old != VTSS_IF_MUX_LIST_WHITE) {
				err = -EINVAL;
				goto ERROR;
			}
			break;

		case VTSS_IF_MUX_LIST_BLACK:
			if (list_old != VTSS_IF_MUX_LIST_WHITE) {
				err = -EINVAL;
				goto ERROR;
			}
			break;

		default:
			err = -EINVAL;
			goto ERROR;
		}
	}

	mutex_lock(&vtss_if_mux_genl_sem);
	list_replace_rcu(&r_old->list, &r_new->list);
	kfree_rcu(r_old, rcu);
	mutex_unlock(&vtss_if_mux_genl_sem);

	return 0;

ERROR:
	kfree(r_new);
	return err;
}

static int vtss_if_mux_genl_cmd_rule_get(struct sk_buff *skb,
					 struct genl_info *info)
{
	printk(KERN_ERR "Not implemented yet: vtss_if_mux_genl_cmd_rule_get\n");
	return -ENOSYS;
}

static int vtss_if_mux_genl_cmd_rule_dump(struct sk_buff *skb,
					  struct netlink_callback *cb)
{
	printk(KERN_ERR
	       "Not implemented yet: vtss_if_mux_genl_cmd_rule_dump\n");
	return -ENOSYS;
}

void debug_dump_element(struct seq_file *s, struct vtss_if_mux_filter_element *e)
{
#define CASE(X)                                                                \
	case VTSS_IF_MUX_FILTER_TYPE_##X:                                      \
		seq_printf(s, "    " #X ": ");                                 \
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
		seq_printf(s, "%02x%02x:%02x%02x:%02x%02x:%02x%02x:"
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

	for (i = 0; i < r->cnt; ++i)
		debug_dump_element(s, &r->elements[i]);
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
	 .policy = vtss_if_mux_genel_policy,
	 // No access control
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_CREATE,
	 .doit = vtss_if_mux_genl_cmd_rule_create,
	 .policy = vtss_if_mux_genel_policy,
	 .flags = GENL_ADMIN_PERM,
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_DELETE,
	 .doit = vtss_if_mux_genl_cmd_rule_delte,
	 .policy = vtss_if_mux_genel_policy,
	 .flags = GENL_ADMIN_PERM,
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_MODIFY,
	 .doit = vtss_if_mux_genl_cmd_rule_modify,
	 .policy = vtss_if_mux_genel_policy,
	 .flags = GENL_ADMIN_PERM,
	},
	{
	 .cmd = VTSS_IF_MUX_GENL_RULE_GET,
	 .doit = vtss_if_mux_genl_cmd_rule_get,
	 .dumpit = vtss_if_mux_genl_cmd_rule_dump,
	 .policy = vtss_if_mux_genel_policy,
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
