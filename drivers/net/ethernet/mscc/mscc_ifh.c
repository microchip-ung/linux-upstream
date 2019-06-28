// SPDX-License-Identifier: (GPL-2.0 OR MIT)
//
// Microsemi SoC FDMA IFH driver
//
// Copyright (c) 2019 Microsemi

#include <linux/dmaengine.h>
#include <linux/etherdevice.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/rtnetlink.h>

#define DEVNAME             "vtss.ifh"
#define ETH_VLAN_TAGSZ     4    /* Size of a 802.1Q VLAN tag */
#define RX_MTU_DEFAULT     (ETH_FRAME_LEN + ETH_FCS_LEN + (2 * ETH_VLAN_TAGSZ))
#define RX_MTU_MIN         64
#define RX_MTU_MAX         16384
#define IF_BUFSIZE_JUMBO   10400
#define SGL_MAX            15

#define FDMA_REQUEST_THRESHOLD     (64*1024)
#define FDMA_REQUEST_MAX           100
#define FDMA_XTR_BUFFER_COUNT      SGL_MAX
#define FDMA_XTR_BUFFER_SIZE       2048

#define VITESSE_ENCAP_SIZE     16
#define VITESSE_ETHTYPE_OFFSET 12
#define VITESSE_IFH_ID_OFFSET  15
#define VITESSE_ETHTYPE_MSB    0x88
#define VITESSE_ETHTYPE_LSB    0x80

#define use_fdma(p)                (p->rxdma && p->txdma)

/* Logging options */

/* Testing options */
// #define PORT_TESTING
// #define DEBUG_RX

struct fdma_config {
	u8 ifh_id;
	u16 ifh_len;
	u16 ifh_encap_len;
};

struct fdma_ifh_priv;

struct fdma_ifh_request {
	struct list_head node;
	struct fdma_ifh_priv *priv;
	enum dma_data_direction direction;
	dma_cookie_t cookie;
	struct sk_buff *skb;
	unsigned int size;
	unsigned int blocks;
	struct scatterlist sgl[SGL_MAX];
	void *buffer[SGL_MAX];
};

struct request_iterator {
	int idx;
	struct fdma_ifh_request *req;
};

struct fdma_ifh_priv {
	struct device *dev;
	struct net_device *netdev;
	const struct fdma_config *config;
	struct dma_chan *rxdma;
	struct dma_chan *txdma;
	struct list_head free_reqs;
	struct list_head tx_reqs;
	struct list_head rx_reqs;
	int available_rx_blocks;
	struct request_iterator curiter;
	int allocations;
	long unsigned tx_packets;
	long unsigned tx_bytes;
	long unsigned rx_packets;
	long unsigned rx_bytes;
	spinlock_t lock;
#ifdef PORT_TESTING
	struct timer_list open_timer;
#endif
};

#ifdef DEBUG_RX
static void dump_byte_array(const char *name, const u8 *buf, size_t len)
{
	char prefix[64];
	if (!buf) {
		return;
	}
	snprintf(prefix, sizeof(prefix), "%.50s[%zu]: ", name, len);
	print_hex_dump(KERN_NOTICE, prefix, DUMP_PREFIX_OFFSET, 16, 1, buf, len,
		       false);
}
#endif

#ifdef PORT_TESTING
/* For test purposes: set ports in up state */
static void open_port(struct timer_list *tmr)
{
	struct fdma_ifh_priv *priv =
		from_timer(priv, tmr, open_timer);

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	rtnl_lock();
	dev_change_flags(priv->netdev, IFF_UP);
	rtnl_unlock();
}

static const u8 packet_template[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  /* Any DMAC */
	0xfe, 0xff, 0xff, 0xff, 0xff, 0xff,  /* Any SMAC */
	0x88, 0x80, /* Ethertype Vitesse */
	0x00, 0x0b, /* IFH ID */
	/* IFH for Fireant */
	0x00, 0x00, 0x11, 0x0c, 0xd2, 0x40, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x02, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x02, 0x00, 0x6c, 0x08, 0xfe, 0x0d, 0x90, 0x88,
	0x00, 0x00, 0x00, 0x00, 
	/* Frame data */
	0x01, 0x80, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x08, 0x00, 0x45, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x11, 0x9b, 0xd9, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* FCS */
	0x6b, 0x9a, 0xec, 0x88,
};

static void fdma_ifh_destruct_test_skb(struct sk_buff *skb)
{
	struct fdma_ifh_priv *priv;

	pr_debug("%s:%d %s: skb head: 0x%px, dev: 0x%px\n", 
		__FILE__, __LINE__, __func__, skb->head, skb->dev);
	priv = netdev_priv(skb->dev);
	priv->allocations--;
	pr_debug("%s:%d %s: allocations: %d\n",
		__FILE__, __LINE__, __func__, priv->allocations);
	kfree(skb->head);
}
#endif

/* Forward Declarations */
static void fdma_ifh_provide_blocks(struct fdma_ifh_priv *priv, 
				      int blocks, int size);


static void fdma_ifh_destruct_skb(struct sk_buff *skb)
{
	struct fdma_ifh_priv *priv;

	pr_debug("%s:%d %s: skb head (not freed here): 0x%px, dev: 0x%px\n", 
		__FILE__, __LINE__, __func__, skb->head, skb->dev);
	priv = netdev_priv(skb->dev);
	priv->allocations--;
	pr_debug("%s:%d %s: allocations: %d\n",
		__FILE__, __LINE__, __func__, priv->allocations);
}

static struct fdma_ifh_request *fdma_ifh_prepare_tx_request(
	struct fdma_ifh_priv *priv,
	enum dma_data_direction dir,
	struct sk_buff *skb)
{
	struct fdma_ifh_request *req;
	struct scatterlist *sg;
	int idx;
	void *data;
	dma_addr_t addr;
	int blocks;
	unsigned int size;

	blocks = skb_shinfo(skb)->nr_frags + 1;
	pr_debug("%s:%d %s: skb: frags: %d, size: %u, headsize: %u\n", 
		__FILE__, __LINE__, __func__,
		skb_shinfo(skb)->nr_frags,
		skb->len,
		skb_headlen(skb));
	if (blocks > SGL_MAX) {
		pr_err("%s:%d %s: too many blocks\n", 
		       __FILE__, __LINE__, __func__);
		return NULL;
	}
	req = list_first_entry_or_null(&priv->free_reqs, struct fdma_ifh_request, 
				       node);
	if (!req) {
		return NULL;
	}
	memset(req->sgl, 0, sizeof(req->sgl));
	memset(req->buffer, 0, sizeof(req->buffer));
	req->priv = priv;
	req->direction = dir;
	req->cookie = 0;
	req->skb = skb;
	req->size = skb->len;
	req->blocks = blocks;
	sg_init_table(req->sgl, blocks);
	sg = req->sgl;

	/* Check encapsulation header (MAC + VLAN Tag) */
	if (!(skb->data[VITESSE_ETHTYPE_OFFSET] == VITESSE_ETHTYPE_MSB &&
		skb->data[VITESSE_ETHTYPE_OFFSET+1] == VITESSE_ETHTYPE_LSB &&
		skb->data[VITESSE_IFH_ID_OFFSET] == priv->config->ifh_id)) {
		pr_debug("%s:%d %s: encapsulation check failed: %d, %d, %d\n",
		       __FILE__, __LINE__, __func__,
		       skb->data[VITESSE_ETHTYPE_OFFSET] == VITESSE_ETHTYPE_MSB,
		       skb->data[VITESSE_ETHTYPE_OFFSET+1] == VITESSE_ETHTYPE_LSB,
		       skb->data[VITESSE_IFH_ID_OFFSET] == priv->config->ifh_id);
		/* Count as dropped */
		priv->netdev->stats.tx_dropped++;
		kfree_skb(skb);
		return NULL;
	}
	/* Skip past the encapsulation header */
	skb_pull_inline(skb, VITESSE_ENCAP_SIZE);
	/* Add room for the FCS  needed_tailroom = ETH_FCS_LEN? */
	skb_put(skb, ETH_FCS_LEN);

	data = skb->data;
	size = skb_headlen(skb); /* Includes the IFH and FCS */
	addr = dma_map_single(priv->dev, data, size, dir);
	if (unlikely(dma_mapping_error(priv->dev, addr))) {
		goto out;
	}
	sg_dma_address(sg) = addr;
	sg_dma_len(sg) = size;
	sg = sg_next(sg);
	/* Add SKB fragments if available */
	for (idx = 0; idx < skb_shinfo(skb)->nr_frags; idx++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[idx];

		if (!skb_frag_size(frag)) {
			sg_dma_address(sg) = 0;
			sg_dma_len(sg) = 0;
			continue;
		}
		addr = skb_frag_dma_map(priv->dev, frag, 0,
					skb_frag_size(frag), dir);
		if (unlikely(dma_mapping_error(priv->dev, addr))) {
			goto out;
		}
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = skb_frag_size(frag);
		sg = sg_next(sg);
	}
	list_move_tail(&req->node, &priv->tx_reqs);
	return req;
out:
	pr_err("%s:%d %s: mapping error\n",
	       __FILE__, __LINE__, __func__);
	return NULL;
}

static struct fdma_ifh_request *fdma_ifh_prepare_rx_request(
	struct fdma_ifh_priv *priv,
	enum dma_data_direction dir,
	int blocks, int size)
{
	struct fdma_ifh_request *req;
	struct scatterlist *sg;
	int idx;
	void *data;
	dma_addr_t addr;

	pr_debug("%s:%d %s: blocks: %u\n", __FILE__, __LINE__, __func__, 
		blocks);
	if (blocks > SGL_MAX) {
		pr_err("%s:%d %s: too many blocks\n", 
		       __FILE__, __LINE__, __func__);
		return 0;
	}
	req = list_first_entry_or_null(&priv->free_reqs, struct fdma_ifh_request, 
				       node);
	if (!req) {
		return NULL;
	}
	memset(req->sgl, 0, sizeof(req->sgl));
	memset(req->buffer, 0, sizeof(req->buffer));
	req->priv = priv;
	req->direction = dir;
	req->cookie = 0;
	req->skb = 0;
	req->size = size;
	size += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	req->blocks = blocks;
	sg_init_table(req->sgl, blocks);
	for_each_sg(req->sgl, sg, blocks, idx) {
		data = kzalloc(size, GFP_KERNEL);
		req->buffer[idx] = data;
		priv->allocations++;
		addr = dma_map_single(priv->dev, data, size, dir);
		if (unlikely(dma_mapping_error(priv->dev, addr))) {
			goto out;
		}
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = req->size;
		pr_debug("%s:%d %s: data: 0x%px, size: %u, shinfo: %lu, phys: 0x%llx\n",
			__FILE__, __LINE__, __func__, 
			data,
			req->size,
			SKB_DATA_ALIGN(sizeof(struct skb_shared_info)),
			addr);
	}
	list_move_tail(&req->node, &priv->rx_reqs);
	return req;
out:
	pr_err("%s:%d %s: mapping error\n",
	       __FILE__, __LINE__, __func__);
	return NULL;
}

static void fdma_ifh_close_request(struct fdma_ifh_priv *priv,
				   struct fdma_ifh_request *req,
				   bool unmap, bool free)
{
	pr_debug("%s:%d %s: [C%u]\n",
		__FILE__, __LINE__, __func__, req->cookie);
	if (unmap) {
		struct scatterlist *sg;
		int idx;

		for_each_sg(req->sgl, sg, req->blocks, idx) {
			dma_unmap_single(priv->dev,
					 sg_dma_address(sg),
					 sg_dma_len(sg),
					 req->direction);
			if (free) {
				pr_debug("%s:%d %s: free: 0x%px [I%u]\n",
					 __FILE__, __LINE__, __func__,
					 req->buffer[idx], idx);
				kfree(req->buffer[idx]);
			}
		}
	}
	list_move_tail(&req->node, &priv->free_reqs);
}

static void init_iterator(struct request_iterator *iter, 
	int idx,
	struct fdma_ifh_request *req)
{
	iter->idx = idx;
	iter->req = req;
	if (idx == iter->req->blocks) {
		iter->idx = 0;
		iter->req = list_next_entry(iter->req, node);
	}
	pr_debug("%s:%d %s: [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx);
}

static void copy_iterator(struct request_iterator *iter,
	struct request_iterator *other)

{
	iter->idx = other->idx;
	iter->req = other->req;
	pr_debug("%s:%d %s: [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx);
}

static struct fdma_ifh_request *next_block(struct request_iterator *iter)
{
	struct fdma_ifh_request *req = NULL;

	iter->idx++;
	if (iter->idx == iter->req->blocks) {
		req = iter->req;
		iter->idx = 0;
		iter->req = list_next_entry(iter->req, node);
	}
	pr_debug("%s:%d %s: [C%u,I%u], req: %u\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx, 
		req != NULL);
	return req;
}

static bool end_of_packet(struct request_iterator *iter,
	struct request_iterator *max)
{
	if (iter->req != max->req) {
		if (iter->idx + 1 == iter->req->blocks) {
			struct fdma_ifh_request *req = list_next_entry(iter->req, node);

			pr_debug("%s:%d %s: %u\n", __FILE__, __LINE__, __func__,
				req == max->req && max->idx == 0);
			return req == max->req && max->idx == 0;
		}
		pr_debug("%s:%d %s: 0\n", __FILE__, __LINE__, __func__);
		return 0;
	}
	pr_debug("%s:%d %s: %u\n", __FILE__, __LINE__, __func__,
		iter->idx + 1 == max->idx);
	return iter->idx + 1 == max->idx;
}

static bool reached(struct request_iterator *iter,
	struct request_iterator *max)
{
	pr_debug("%s:%d %s: %u\n", __FILE__, __LINE__, __func__,
		iter->req == max->req && iter->idx == max->idx);
	return iter->req == max->req && iter->idx == max->idx;
}

static void *get_block_data(struct fdma_ifh_priv *priv, struct request_iterator *iter)
{
	struct scatterlist *sg = &iter->req->sgl[iter->idx];

	dma_unmap_single(priv->dev,
			 sg_dma_address(sg),
			 sg_dma_len(sg),
			 iter->req->direction);
	pr_debug("%s:%d %s: [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, iter->req->cookie, iter->idx);
	return iter->req->buffer[iter->idx];
}

static struct sk_buff *create_receive_skb(
	struct fdma_ifh_priv *priv,
	struct request_iterator *iter,
	struct request_iterator *max,
	int *blks,
	u32 size)
{
	struct sk_buff *skb = 0;
	u32 *packet;
	void *data;
	unsigned int blocks;
	unsigned int block_bytes;
	struct fdma_ifh_request *done_req;

	pr_debug("%s:%d %s: from: [C%u,I%u] to: [C%u,I%u]\n", 
		 __FILE__, __LINE__, __func__,
		 iter->req->cookie,
		 iter->idx,
		 max->req->cookie,
		 max->idx);

	packet = data = get_block_data(priv, iter);
	/* Get the packet size (includes IFH and FCS) */
	blocks = DIV_ROUND_UP(size, iter->req->size);
	block_bytes = blocks * iter->req->size;
	pr_debug("%s:%d %s: data: 0x%px, bytes: %u, size: %u, blocks: %u, block_bytes: %u\n",
		 __FILE__, __LINE__, __func__, 
		 data,
		 size,
		 iter->req->size,
		 blocks,
		 block_bytes);
	if (end_of_packet(iter, max)) {
		skb = build_skb(data, 0);
		if (!skb) {
			pr_err("%s:%d %s: no skb: %u bytes\n", 
			       __FILE__, __LINE__, __func__, iter->req->size);
#ifdef DEBUG
			dump_byte_array("RxData", data, min(block_bytes, iter->req->size));
#endif
			goto out;
		}

#ifdef DEBUG_RX
		dump_byte_array("IFH RxData", data, min(size, iter->req->size));
#endif
		skb->dev = priv->netdev;
		skb->destructor = fdma_ifh_destruct_skb;
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n",
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		skb_put(skb, size);
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n", 
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		done_req = next_block(iter);
		if (done_req) {
			pr_debug("%s:%d %s: done: [C:%u]\n", 
				 __FILE__, __LINE__, __func__, done_req->cookie);
			list_move_tail(&done_req->node, &priv->free_reqs);
		}
	} else {
		void *ptr;

		skb = netdev_alloc_skb(priv->netdev, block_bytes);
		if (!skb) {
			pr_err("%s:%d %s: no skb: %u bytes\n", 
			       __FILE__, __LINE__, __func__, block_bytes);
			goto out_free;
		}

		ptr = skb->data;
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n",
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		skb_put(skb, size);
		pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n", 
			 __FILE__, __LINE__, __func__, skb->len, skb->data);
		while (!reached(iter, max)) {
			data = get_block_data(priv, iter);
#ifdef DEBUG
			dump_byte_array("RxData", data, iter->req->size);
#endif
			pr_debug("%s:%d %s: copy: len: %d, data: 0x%px\n", 
				 __FILE__, __LINE__, __func__, 
				 iter->req->size, data);
			memcpy(ptr, data, iter->req->size);
			kfree(data);
			priv->allocations--;
			done_req = next_block(iter);
			if (done_req) {
				pr_debug("%s:%d %s: done: [C:%u]\n", 
					 __FILE__, __LINE__, __func__, done_req->cookie);
				list_move_tail(&done_req->node, &priv->free_reqs);
			}
			ptr += iter->req->size;
		}
	}
	*blks = blocks;
	skb->protocol = eth_type_trans(skb, priv->netdev);
	pr_debug("%s:%d %s: skb: len: %d, data: 0x%px\n", 
		 __FILE__, __LINE__, __func__, skb->len, skb->data);
	return skb;

out_free:
	while (!reached(iter, max)) {
#ifdef DEBUG
		dump_byte_array("RxData", data, min(block_bytes, iter->req->size));
#endif
		pr_debug("%s:%d %s: free: 0x%px\n",
			 __FILE__, __LINE__, __func__, data);
		kfree(data);
		priv->allocations--;
		done_req = next_block(iter);
		if (done_req) {
			pr_debug("%s:%d %s: done: [C:%u]\n", 
				 __FILE__, __LINE__, __func__, done_req->cookie);
			list_move_tail(&done_req->node, &priv->free_reqs);
		}
		data = get_block_data(priv, iter);
	}
out:
	*blks = 0;
	return 0;
}

static void fdma_ifh_receive_blocks_cb(void *data, 
	const struct dmaengine_result *result)
{
	struct fdma_ifh_request *req = data;
	struct fdma_ifh_priv *priv;
	int used_blocks;
	enum dma_status status;
	struct dma_tx_state state;
	struct sk_buff *skb;
	struct request_iterator next;
	int next_sof;

	pr_debug("%s:%d %s: result: %u, residue: %u\n", 
		__FILE__, __LINE__, __func__, 
		result->result, result->residue);
	if (!req) {
		pr_err("%s:%d %s: no request\n",
			__FILE__, __LINE__, __func__);
		return;
	}
	priv = req->priv;
	status = dmaengine_tx_status(priv->rxdma, req->cookie, &state);
	next_sof = req->blocks - (state.residue / req->size);
	init_iterator(&next, next_sof, req);
	pr_debug("%s:%d %s: status: %u, state: (L%u, U%u), size: %u, [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, 
		status,
		state.last, state.used, state.residue,
		req->cookie,
		next_sof);

	if (result->result == DMA_TRANS_NOERROR) {
		skb = create_receive_skb(priv, &priv->curiter, &next, 
			&used_blocks, result->residue);
		if (!skb) {
			pr_err("%s:%d %s: could not create skb: [C%u,I%u]\n", 
				__FILE__, __LINE__, __func__, 
				priv->curiter.req->cookie,
				priv->curiter.idx);
			goto err;
		}
	} else {
		goto err;
	}
	/* Save state for next packet */
	copy_iterator(&priv->curiter, &next);
	pr_debug("%s:%d %s: skb: len: %d, data: 0x%px, used_blocks: %u\n", 
		__FILE__, __LINE__, __func__, skb->len, skb->data, used_blocks);

	priv->rx_packets++;
	priv->rx_bytes += skb->len;

	pr_debug("%s:%d %s: RX Done: packets: %lu, bytes: %lu, allocations: %d\n", 
		__FILE__, __LINE__, __func__, 
		priv->rx_packets,
		priv->rx_bytes,
		priv->allocations);

#ifdef DEBUG
	dump_byte_array("RxSkb", skb->data, skb->len);
#endif
	netif_rx(skb);

	priv->available_rx_blocks -= used_blocks;
	fdma_ifh_provide_blocks(priv, req->blocks, req->size);
	return;
err:
	pr_err("%s:%d %s: error: %u, [C%u,I%u]\n", 
		__FILE__, __LINE__, __func__, 
		result->result, next.req->cookie, next.idx);
	/* TODO: count unreceived bytes/packet */
	/* Save state for next packet */
	copy_iterator(&priv->curiter, &next);
}

static int fdma_ifh_receive_blocks(struct fdma_ifh_priv *priv, int blocks, int size)
{
	struct dma_async_tx_descriptor *txd;
	struct fdma_ifh_request *req;

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	req = fdma_ifh_prepare_rx_request(priv, DMA_FROM_DEVICE,
				       blocks, size);
	if (!req) {
		pr_err("%s:%d %s: no more requests\n",
			__FILE__, __LINE__, __func__);
		return 0;
	}
	priv->available_rx_blocks += blocks;
	if (!priv->curiter.req) {
		init_iterator(&priv->curiter, 0, req);
	}
	txd = dmaengine_prep_slave_sg(priv->rxdma, req->sgl, blocks,
				      DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);

	if (!txd) {
		dev_err(priv->dev, "Could not get RX Descriptor\n");
		goto unmap;
	}

	pr_debug("%s:%d %s: txd: 0x%px\n", 
		__FILE__, __LINE__, __func__, txd);
	txd->callback_param = req;
	txd->callback_result = fdma_ifh_receive_blocks_cb;
	req->cookie = dmaengine_submit(txd);
	if (req->cookie < DMA_MIN_COOKIE) {
		dev_err(priv->dev, "Submit failed\n");
		goto unmap;
	}
	pr_debug("%s:%d %s: Submitted: %d\n", __FILE__, __LINE__, __func__, req->cookie);
	dma_async_issue_pending(priv->rxdma);
	return 1;

unmap:
	pr_err("%s:%d %s: error, close request\n", __FILE__, __LINE__, __func__);
	fdma_ifh_close_request(priv, req, 1, 1);
	return 0;
}

static void fdma_ifh_provide_blocks(struct fdma_ifh_priv *priv, 
				      int blocks, int size)
{
	pr_debug("%s:%d %s: blocks: %u, size: %u\n", 
		__FILE__, __LINE__, __func__, blocks, size);
	while (priv->available_rx_blocks * size < FDMA_REQUEST_THRESHOLD) {
		fdma_ifh_receive_blocks(priv, blocks, size);
	}
	pr_debug("%s:%d %s: available blocks: %d\n", 
		__FILE__, __LINE__, __func__, priv->available_rx_blocks);
}

static void fdma_ifh_transmit_cb(void *data, 
	const struct dmaengine_result *result)
{
	struct fdma_ifh_request *req = data;
	enum dma_status status;
	struct dma_tx_state state;
	struct fdma_ifh_priv *priv;

	pr_debug("%s:%d %s: result: %u\n", 
		__FILE__, __LINE__, __func__, 
		result->result);

	if (!req) {
		return;
	}
	priv = req->priv;
	if (result->result != DMA_TRANS_NOERROR) {
		pr_err("%s:%d %s: error: %u, [C%u]\n", 
			__FILE__, __LINE__, __func__, 
			result->result, req->cookie);
	} else {
		priv->tx_packets++;
		status = dmaengine_tx_status(priv->txdma, req->cookie, &state);
		pr_debug("%s:%d %s: status %d, state: last: %u, used: %u, residue: %u\n", 
			__FILE__, __LINE__, __func__, 
			status, state.last, state.used, state.residue);
		/* TODO: req->size should cover the whole thing */
		priv->tx_bytes += req->blocks * req->size;
	}
	fdma_ifh_close_request(priv, req, 1, 0);
	dev_consume_skb_any(req->skb);

	pr_debug("%s:%d %s: TX Done: packets: %lu, bytes: %lu, allocations: %d\n", 
		__FILE__, __LINE__, __func__, 
		priv->tx_packets,
		priv->tx_bytes,
		priv->allocations);
}

static int fdma_ifh_transmit(struct fdma_ifh_priv *priv, struct sk_buff *skb)
{
	struct dma_async_tx_descriptor *txd;
	struct fdma_ifh_request *req;

	req = fdma_ifh_prepare_tx_request(priv, DMA_TO_DEVICE, skb);
	if (!req) {
		pr_debug("%s:%d %s: request prepare error\n",
			__FILE__, __LINE__, __func__);
		return 0;
	}
	txd = dmaengine_prep_slave_sg(priv->txdma, req->sgl, req->blocks,
				      DMA_MEM_TO_DEV, 0);

	if (!txd) {
		dev_err(priv->dev, "Could not get TX Descriptor\n");
		goto unmap;
	}

	pr_debug("%s:%d %s: txd: 0x%px\n", 
		__FILE__, __LINE__, __func__, txd);
	txd->callback_param = req;
	txd->callback_result = fdma_ifh_transmit_cb;
	req->cookie = dmaengine_submit(txd);
	if (req->cookie < DMA_MIN_COOKIE) {
		dev_err(priv->dev, "Submit failed\n");
		goto unmap;
	}
	pr_debug("%s:%d %s: Submitted: %d\n", __FILE__, __LINE__, __func__, req->cookie);
	dma_async_issue_pending(priv->txdma);
	return 1;

unmap:
	pr_err("%s:%d %s: error, close request\n", __FILE__, __LINE__, __func__);
	fdma_ifh_close_request(priv, req, 1, 0);
	dev_consume_skb_any(req->skb);
	return 0;
}

static void packet_generic_netlink_init(void)
{
}

static void packet_generic_netlink_uninit(void)
{
}

static int mscc_fdma_ifh_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}


static int mscc_fdma_ifh_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static int mscc_fdma_ifh_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct fdma_ifh_priv *priv = netdev_priv(dev);

	if (!use_fdma(priv)) {
		pr_err("%s: no FDMA\n", dev->name);
		dev->stats.tx_dropped++;
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	pr_debug("%s: Transmit %d bytes: 0x%px\n", dev->name, skb->len, skb->data);

#ifdef DEBUG
	dump_byte_array("TxSkb", skb->data, skb->len);
#endif
	/* TODO: Check and remove encapsulation header */

	fdma_ifh_transmit(priv, skb);
	return NETDEV_TX_OK;
}

static int mscc_fdma_ifh_change_mtu(struct net_device *dev, int new_mtu)
{
	struct fdma_ifh_priv *priv = netdev_priv(dev);

	if (new_mtu < (RX_MTU_MIN + ETH_VLAN_TAGSZ + ETH_FCS_LEN + 
		       priv->config->ifh_len) || new_mtu > IF_BUFSIZE_JUMBO) {
		return -EINVAL;
	}
	dev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops fdma_ifh_netdev_ops = {
	.ndo_open            = mscc_fdma_ifh_open,
	.ndo_stop            = mscc_fdma_ifh_close,
	.ndo_start_xmit      = mscc_fdma_ifh_start_xmit,
	.ndo_change_mtu      = mscc_fdma_ifh_change_mtu,
	.ndo_validate_addr   = eth_validate_addr,
	.ndo_set_mac_address = eth_mac_addr,
};

static struct net_device *ifhdev_create(struct platform_device *pdev)
{
	struct fdma_ifh_priv *priv;
	struct net_device *dev;

	if ((dev = devm_alloc_etherdev(&pdev->dev, sizeof(*priv))) == NULL) {
		return NULL;
	}

	dev->netdev_ops = &fdma_ifh_netdev_ops;
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof(*priv));
	priv->netdev = dev; /* Backlink */
	priv->dev = &pdev->dev;
	priv->config = device_get_match_data(&pdev->dev);
	/* This device adds no MAC header - it must be part of data */
	// dev->hard_header_len = dev->min_header_len = 0;

	eth_hw_addr_random(dev);
	memset(&dev->broadcast[0], 0xff, 6);

	/* Set arbitrarily high for direct injection (not rx) */
	dev->mtu = IF_BUFSIZE_JUMBO;

	spin_lock_init(&priv->lock);

	return dev;
}

static int mscc_fdma_ifh_probe(struct platform_device *pdev)
{
	struct fdma_ifh_priv *priv;
	struct net_device *dev;
	int idx;
	int ret;

	if ((dev = ifhdev_create(pdev)) == NULL) {
		dev_info(&pdev->dev, "Cannot create netdevice\n");
		return -ENOMEM;
	}
	strcpy(dev->name, DEVNAME);

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);
	priv = netdev_priv(dev);

	/* Request TX and RX DMA Channels */
	priv->txdma = dma_request_chan(priv->dev, "tx");
	priv->rxdma = dma_request_chan(priv->dev, "rx");
	if (IS_ERR(priv->txdma) || IS_ERR(priv->rxdma)) {
		if (!IS_ERR(priv->rxdma))
			dma_release_channel(priv->rxdma);
		if (!IS_ERR(priv->txdma))
			dma_release_channel(priv->txdma);
		priv->rxdma = NULL;
		priv->txdma = NULL;
		/* TODO: Use register access */
		dev_warn(priv->dev, "No FDMA support: retry later\n");
		return -EPROBE_DEFER;
	} else {
		dev_info(priv->dev, "Requested TX & RX DMA channels\n");
	}

	INIT_LIST_HEAD(&priv->free_reqs);
	INIT_LIST_HEAD(&priv->rx_reqs);
	INIT_LIST_HEAD(&priv->tx_reqs);
	for (idx = 0; idx < FDMA_REQUEST_MAX; ++idx) {
		struct fdma_ifh_request *req =
			devm_kzalloc(priv->dev, sizeof(*req), GFP_KERNEL);
		if (!req) {
			goto err_list;
		}
		list_add(&req->node, &priv->free_reqs);
	}

	if (use_fdma(priv)) {
		fdma_ifh_provide_blocks(priv, FDMA_XTR_BUFFER_COUNT, 
					  FDMA_XTR_BUFFER_SIZE);
	}

	dev->needed_headroom = priv->config->ifh_encap_len + priv->config->ifh_len;
	dev->needed_tailroom = ETH_FCS_LEN;

	ret = register_netdev(dev);
	if (ret < 0) {
		dev_err(priv->dev, "Cannot register netdevice: %d\n", ret);
		free_netdev(dev);
		return ret;
	}

	packet_generic_netlink_init();

#ifdef PORT_TESTING
	/* For test purposes: Open the port in 5s */
	timer_setup(&priv->open_timer, open_port, 0);
	priv->open_timer.expires = jiffies + msecs_to_jiffies(10000);
	add_timer(&priv->open_timer);
#endif
	return 0;

err_list:
	dev_err(priv->dev, "Request allocation error\n");
	return -ENOMEM;
}

static int mscc_fdma_ifh_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct fdma_ifh_priv *priv = netdev_priv(dev);

	dev_info(priv->dev, "removing\n");

	packet_generic_netlink_uninit();

	dma_release_channel(priv->rxdma);
	dma_release_channel(priv->txdma);

	unregister_netdev(dev);
	free_netdev(dev);
	return 0;
}

static const struct fdma_config fireant_data = {
	.ifh_id = 0xb,
	.ifh_len = 36,
	.ifh_encap_len = 16,
};

static const struct of_device_id mscc_fdma_ifh_match[] = {
	{ .compatible = "mscc,vsc7558-fdma-ifh", .data = &fireant_data },
	{},
};
MODULE_DEVICE_TABLE(of, mscc_fdma_ifh_match);

static struct platform_driver mscc_fdma_ifh_driver = {
	.probe = mscc_fdma_ifh_probe,
	.remove = mscc_fdma_ifh_remove,
	.driver = {
		.name = "mscc-fdma-ifh",
		.of_match_table = mscc_fdma_ifh_match,
	},
};

module_platform_driver(mscc_fdma_ifh_driver);

MODULE_AUTHOR("Steen Hegelund <steen.hegelund@microchip.com>");
MODULE_DESCRIPTION("Microsemi FDMA IFH driver");
MODULE_LICENSE("Dual MIT/GPL");
