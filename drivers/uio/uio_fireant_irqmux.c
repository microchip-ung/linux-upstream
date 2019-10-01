// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Microsemi Fireant UIO driver
 *
 * Author: <lars.povlsen@microchip.com>
 * License: Dual MIT/GPL
 * Copyright (c) 2019 Microchip Corporation
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/of.h>

/* Software IRQ ctl registers */
#define REGOFF_SHARED_INTR	0x00000098
#define REGOFF_SHARED_INTR_CLR	0x0000009c
#define REGOFF_SHARED_INTR_SET	0x000000a0

#define DRIVER_NAME	"uio_fireant_irqmux"
#define MAXNAMELEN	16

struct slave_irq {
	int index;
	int irq;
	int active;
	char name[MAXNAMELEN];
	struct irqmux_platdata *priv;
};

struct irqmux_platdata {
	struct uio_info info;
	spinlock_t lock;
	unsigned long flags;
	struct platform_device *pdev;
	struct slave_irq *sirq;
	int n_sirq;
	int n_active;
};

/* Bits in irqmux_platdata.flags */
enum {
	UIO_IRQ_DISABLED = 0,
};

#if defined(DEBUG)
static ssize_t swint_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct irqmux_platdata *priv = platform_get_drvdata(pdev);
	u32 val;

	val = readl(priv->info.mem[0].internal_addr + REGOFF_SHARED_INTR);

	return sprintf(buf, "0x%x\n", val);
}

static ssize_t swint_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct irqmux_platdata *priv = platform_get_drvdata(pdev);
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	writel(val, priv->info.mem[0].internal_addr + REGOFF_SHARED_INTR);

	return count;
}

static DEVICE_ATTR_RW(swint);
#endif

static ssize_t irqctl_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct irqmux_platdata *priv = platform_get_drvdata(pdev);
	int i;
	ssize_t n, nwritten = 0;

	for (i = 0; i < priv->n_sirq; i++) {
		if (priv->sirq[i].active) {
			n = sprintf(buf, "%d|%s\n", i, priv->sirq[i].name);
			nwritten += n;
			buf += n;
		}
	}

#if defined(DEBUG)
	nwritten += sprintf(buf, "Active: %d\n", priv->n_active);
#endif

	return nwritten;
}

static ssize_t irqctl_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct irqmux_platdata *priv = platform_get_drvdata(pdev);
	int index;
	unsigned long flags;

	if (kstrtoint(buf, 0, &index))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);
	/* Allow [1; max-1] - 0 is master (reserved) */
	if (index > 0 && index < priv->n_sirq) {
		if (priv->sirq[index].active) {
			dev_dbg(&priv->pdev->dev, "Clear IRQ %d = %s, #active = %d\n",
				index, priv->sirq[index].name,
				priv->n_active);
			priv->sirq[index].active = false;
			priv->n_active--;
			if (priv->n_active == 0) {
				/* Clear combined mastr IRQ SW0 */
				writel(BIT(0), priv->info.mem[0].internal_addr +
				       REGOFF_SHARED_INTR_CLR);
			}
			enable_irq(priv->sirq[index].irq);
		} else {
			dev_warn(&priv->pdev->dev,
				 "Interrupt %d already inactive\n", index);
		}
	} else {
		dev_warn(&priv->pdev->dev, "Illegal interrupt index: %d\n",
			 index);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return count;
}

static DEVICE_ATTR_RW(irqctl);

static struct attribute *irqmux_attrs[] = {
#if defined(DEBUG)
	&dev_attr_swint.attr,
#endif
	&dev_attr_irqctl.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = irqmux_attrs,
};

static int uio_fireant_irqmux_open(struct uio_info *info, struct inode *inode)
{
	return 0;
}

static int uio_fireant_irqmux_release(struct uio_info *info,
				      struct inode *inode)
{
	//struct irqmux_platdata *priv = info->priv;
	return 0;
}

static irqreturn_t uio_fireant_irqmux_handler(int irq,
					      struct uio_info *dev_info)
{
	struct irqmux_platdata *priv = dev_info->priv;

	/* Just disable the interrupt in the interrupt controller, and
	 * remember the state so we can allow user space to enable it later.
	 */

	spin_lock(&priv->lock);
	if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
		disable_irq_nosync(irq);
	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int uio_fireant_irqmux_irqcontrol(struct uio_info *dev_info, s32 irq_on)
{
	struct irqmux_platdata *priv = dev_info->priv;
	unsigned long flags;

	/* Allow user space to enable and disable the interrupt
	 * in the interrupt controller, but keep track of the
	 * state to prevent per-irq depth damage.
	 *
	 * Serialize this operation to support multiple tasks and concurrency
	 * with irq handler on SMP systems.
	 */

	spin_lock_irqsave(&priv->lock, flags);
	if (irq_on) {
		if (__test_and_clear_bit(UIO_IRQ_DISABLED, &priv->flags))
			enable_irq(dev_info->irq);
	} else {
		if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
			disable_irq_nosync(dev_info->irq);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static irqreturn_t slave_irq(int irq, void *ident)
{
	struct slave_irq *sirq = ident;
	struct irqmux_platdata *priv = sirq->priv;
	unsigned long flags;

	dev_dbg(&priv->pdev->dev, "IRQ %d: %s fired\n", irq, sirq->name);

	spin_lock_irqsave(&priv->lock, flags);
	if (!sirq->active) {
		sirq->active = true;
		priv->n_active++;
		disable_irq_nosync(irq);
		/* Trigger SW0 */
		writel(BIT(0), priv->info.mem[0].internal_addr +
		       REGOFF_SHARED_INTR_SET);
	} else {
		dev_err(&priv->pdev->dev, "Got IRQ %d while already active\n",
			sirq->index);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return IRQ_HANDLED;
}

static int uio_fireant_irqmux_request_irqs(struct irqmux_platdata *priv)
{
	struct platform_device *pdev = priv->pdev;
	int irq, num, max;

	priv->n_sirq = platform_irq_count(pdev);
	priv->sirq = devm_kzalloc(&pdev->dev,
				  sizeof(struct slave_irq)*priv->n_sirq,
				  GFP_KERNEL);
	if (!priv->sirq)
		return -ENOMEM;

	for (num = 0, max = priv->n_sirq; num < max; num++) {
		struct resource *r;

		r = platform_get_resource(pdev, IORESOURCE_IRQ, num);
		if (!r)
			return -ENXIO;
		irq = platform_get_irq(pdev, num);
		priv->sirq[num].index = num;
		priv->sirq[num].irq = irq;
		priv->sirq[num].priv = priv;
		strncpy(priv->sirq[num].name, r->name, MAXNAMELEN);
		if (irq <= 0) {
			dev_err(&pdev->dev, "failed to get IRQ %d\n", num);
			return irq;
		}
		if (num == 0) {
			priv->info.irq = irq;
			if (strcmp(r->name, "master")) {
				dev_err(&pdev->dev,
					"First irq must be 'master'\n");
				return -ENXIO;
			}
		} else {
			int ret = request_irq(irq, slave_irq, 0, r->name,
					      (void *) &priv->sirq[num]);
			if (ret < 0) {
				dev_err(&pdev->dev, "can not get IRQ %d\n",
					irq);
				return -ENXIO;
			}
		}
	}

	return 0;
}

static int uio_fireant_irqmux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct irqmux_platdata *priv;
	int ret = -EINVAL;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	priv->flags = 0; /* interrupt is enabled to begin with */
	priv->pdev = pdev;

	ret = uio_fireant_irqmux_request_irqs(priv);
	if (ret) {
		dev_err(dev, "failed to get IRQs\n");
		return ret;
	}

	priv->info.name = pdev->dev.of_node->name;
	priv->info.version = "devicetree";
	priv->info.mem[0].memtype = UIO_MEM_PHYS;
	priv->info.mem[0].addr = pdev->resource[0].start;
	priv->info.mem[0].size = resource_size(&(pdev->resource[0]));
	priv->info.mem[0].name = pdev->resource[0].name;
	priv->info.mem[0].internal_addr = ioremap(priv->info.mem[0].addr,
						  priv->info.mem[0].size);
	if (!priv->info.mem[0].internal_addr) {
		dev_err(dev, "failed to map chip region\n");
		return -ENODEV;
	}

	priv->info.handler = uio_fireant_irqmux_handler;
	priv->info.irqcontrol = uio_fireant_irqmux_irqcontrol;
	priv->info.open = uio_fireant_irqmux_open;
	priv->info.release = uio_fireant_irqmux_release;
	priv->info.priv = priv;

	ret = uio_register_device(dev, &priv->info);
	if (ret) {
		dev_err(dev, "unable to register uio device\n");
		return ret;
	}

	if (sysfs_create_group(&dev->kobj, &attr_group))
		dev_err(dev, "sysfs register error\n");

	platform_set_drvdata(pdev, priv);
	return 0;
}

static int uio_fireant_irqmux_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct irqmux_platdata *priv = platform_get_drvdata(pdev);

	uio_unregister_device(&priv->info);

	iounmap(priv->info.mem[0].internal_addr);

	priv->info.handler = NULL;
	priv->info.irqcontrol = NULL;

	sysfs_remove_group(&dev->kobj, &attr_group);

	return 0;
}

static const struct of_device_id uio_of_fireant_irqmux_match[] = {
	{ .compatible = "mscc,uio_fireant_irqmux", },
	{},
};

static struct platform_driver uio_fireant_irqmux = {
	.probe = uio_fireant_irqmux_probe,
	.remove = uio_fireant_irqmux_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(uio_of_fireant_irqmux_match),
	},
};

module_platform_driver(uio_fireant_irqmux);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars Povlsen <lars.povlsen@microchip.com>");
