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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/of_platform.h>

#include "vtss_dying_gasp.h"

struct vtss_dying_gasp_private {
	struct net_device *ndev;
	struct tasklet_struct tasklet;
};

/* dying gasp link list, defined genetlink file */
extern struct list_head VTSS_DYING_GASP_GENL_BUF;

/* Dying gasp transmit tasklet */
static void vtss_dying_gasp_tasklet(unsigned long data)
{
	struct vtss_dying_gasp_private *priv =
		(struct vtss_dying_gasp_private *)data;
	struct vtss_dying_gasp_genl_buf *b;
	struct sk_buff *skb_dying_gasp;

	/* Get vtss.ifh in the isr, so there is no dependency on the order
	 * of module initialization */
	if (!priv->ndev) {
		priv->ndev = dev_get_by_name(&init_net, "vtss.ifh");
	}

	pr_debug("%s: net_device: %px\n", __func__, priv->ndev);
	/* Send dying gasp pdu only if vtss.ifh interface is available */
	if (priv->ndev && priv->ndev->netdev_ops->ndo_start_xmit) {
		rcu_read_lock();
		list_for_each_entry_rcu (b, &VTSS_DYING_GASP_GENL_BUF, list) {
			/* allocate a sk_buff for dying gasp buf */
			skb_dying_gasp = alloc_skb(ETH_FRAME_LEN, GFP_ATOMIC);
			if (skb_dying_gasp) {
				skb_dying_gasp->len = b->msg_len;
				memcpy(skb_dying_gasp->data, (unsigned char *)b->msg, b->msg_len);

				pr_debug("%s: TX: %px: skb: %px\n", __func__, priv->ndev, skb_dying_gasp);
				priv->ndev->netdev_ops->ndo_start_xmit(
					skb_dying_gasp, priv->ndev);
				/* skb_dying_gasp will be freed after transmit */
			}
		}
		rcu_read_unlock();
	}
}

/* Dying gasp interrupt handler */
static irqreturn_t vtss_dying_gasp_isr(int irq, void *data)
{
	struct vtss_dying_gasp_private *priv = data;

	/* Net device TX cannot be done from an ISR */
	tasklet_schedule(&priv->tasklet);
	/* Disable IRQ, otherwise FDMA IRQ would be serviced */
	pr_debug("%s: irq: %d\n", __func__, irq);
	disable_irq_nosync(irq);
	return IRQ_HANDLED;
}

static int vtss_dying_gasp_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	struct vtss_dying_gasp_private *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "Allocation failed\n");
		goto exit;
	}
	platform_set_drvdata(pdev, priv);
	priv->ndev = dev_get_by_name(&init_net, "vtss.ifh");

	ret = vtss_dying_gasp_genetlink_init();
	if (ret < 0) {
		dev_err(&pdev->dev, "Netlink failed\n");
		goto exit;
	}

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, vtss_dying_gasp_isr, 0,
		dev_name(&pdev->dev), priv);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register IRQ %d\n", irq);
		goto exit;
	}

#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
	/* PCB116: GPIO#04, ALT"01", IRQ0_IN */
	vcoreiii_gpio_set_mode( 4, 1);
#endif
	tasklet_init(&priv->tasklet, vtss_dying_gasp_tasklet,
		     (unsigned long)priv);

	dev_info(&pdev->dev, "Ready on irq: %d\n", irq);
	/* return zero on success */
	return 0;

exit:
	vtss_dying_gasp_genetlink_uninit();
	return ret;
}

static int vtss_dying_gasp_remove(struct platform_device *pdev)
{
	vtss_dying_gasp_genetlink_uninit();
	return 0;
}

static const struct of_device_id vtss_dying_gasp_match[] = {
	{ .compatible = "mscc,serval2-dying-gasp" },
	{},
};
MODULE_DEVICE_TABLE(of, vtss_dying_gasp_match);

static struct platform_driver vtss_dying_gasp_driver = {
	.probe = vtss_dying_gasp_probe,
	.remove = vtss_dying_gasp_remove,
	.driver = {
		.name = "vtss_dying_gasp",
		.of_match_table = vtss_dying_gasp_match,
	},
};

module_platform_driver(vtss_dying_gasp_driver);

MODULE_AUTHOR("Wenxi Jin <wenxi.jin@microsemi.com>");
MODULE_DESCRIPTION("Dying gasp driver");
MODULE_LICENSE("GPL v2");
