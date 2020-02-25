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

#define DRIVER_NAME	"uio_fireant_irqmux"
#define MAXNAMELEN	16

struct slave_irq_data {
	int index;
	int irq;
	int active;
	char name[MAXNAMELEN];
	struct irqmux_platdata *priv;
};

struct uio_fireant_irqmux_config {
	u32 reg_off_ident;
	u32 reg_off_force;
	u32 reg_off_clr;
	u32 reg_off_set;
	u32 master_mask;
};

struct irqmux_platdata {
	struct uio_info info;
	spinlock_t lock;
	const struct uio_fireant_irqmux_config *cfg;
	unsigned long flags;
	struct platform_device *pdev;
	struct slave_irq_data *sirq;
	int n_sirq;
	int n_active;
	bool io_enabled;
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
	u32 val = 0;

	if (priv->io_enabled) {
		val = readl(priv->info.mem[0].internal_addr + priv->cfg->reg_off_ident);
	}

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

	if (priv->io_enabled) {
		writel(val, priv->info.mem[0].internal_addr + priv->cfg->reg_off_force);
	}

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
			if (priv->io_enabled && priv->n_active == 0) {
				/* Clear combined master IRQ */
				writel(priv->cfg->master_mask,
				       priv->info.mem[0].internal_addr +
				       priv->cfg->reg_off_clr);
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
	struct slave_irq_data *sirq = ident;
	struct irqmux_platdata *priv = sirq->priv;
	unsigned long flags;

	dev_dbg(&priv->pdev->dev, "IRQ %d: %s fired\n", irq, sirq->name);

	spin_lock_irqsave(&priv->lock, flags);
	if (!sirq->active) {
		sirq->active = true;
		priv->n_active++;
		disable_irq_nosync(irq);
		if (priv->io_enabled) {
			/* Trigger SW0 */
			writel(priv->cfg->master_mask, priv->info.mem[0].internal_addr +
			       priv->cfg->reg_off_set);
		}
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
	struct device *dev = &pdev->dev;
	int irq, num, max;

	priv->n_sirq = platform_irq_count(pdev);
	priv->sirq = devm_kzalloc(dev,
				  sizeof(struct slave_irq_data)*priv->n_sirq,
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
			dev_err(dev, "failed to get IRQ %d\n", num);
			return irq;
		}
		if (num == 0) {
			priv->info.irq = irq;
			if (strcmp(r->name, "master")) {
				dev_err(dev, "First irq must be 'master'\n");
				return -ENXIO;
			}
		} else {
			int ret = request_irq(irq, slave_irq, 0, r->name,
					      (void *) &priv->sirq[num]);
			if (ret < 0) {
				dev_err(dev, "can not get IRQ %d\n", irq);
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
	priv->cfg = device_get_match_data(&pdev->dev);

	priv->io_enabled = !of_property_read_bool(pdev->dev.of_node,
						  "external-cpu");
	if (priv->io_enabled) {
		dev_info(dev, "IO is enabled\n");
	} else {
		dev_info(dev, "IO is disabled, using external CPU\n");
	}

	ret = uio_fireant_irqmux_request_irqs(priv);
	if (ret) {
		dev_err(dev, "failed to get IRQs\n");
		return ret;
	}

	priv->info.name = pdev->dev.of_node->name;
	priv->info.version = "devicetree";
	if (priv->io_enabled) {
		int i;
		const struct resource *res;

		for(i = 0, res = &pdev->resource[0]; resource_type(res) == IORESOURCE_MEM; i++, res++) {
			size_t sz = resource_size(res);
			priv->info.mem[i].memtype = UIO_MEM_PHYS;
			priv->info.mem[i].addr = res->start;
			priv->info.mem[i].size = sz;
			priv->info.mem[i].name = res->name;
			priv->info.mem[i].internal_addr = devm_ioremap(dev,
								       priv->info.mem[i].addr,
								       priv->info.mem[i].size);
			if (!priv->info.mem[i].internal_addr) {
				dev_err(dev, "failed to map chip region %d sz %zd\n", i, sz);
				return -ENODEV;
			}
		}
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
	dev_info(dev, "Mapping %pR\n", &pdev->resource[0]);
	return 0;
}

static int uio_fireant_irqmux_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct irqmux_platdata *priv = platform_get_drvdata(pdev);

	uio_unregister_device(&priv->info);

	priv->info.handler = NULL;
	priv->info.irqcontrol = NULL;

	sysfs_remove_group(&dev->kobj, &attr_group);

	return 0;
}

static const struct uio_fireant_irqmux_config fireant_data = {
	.reg_off_ident = 0x00000098,
	.reg_off_force = 0x00000098,
	.reg_off_clr   = 0x0000009c,
	.reg_off_set   = 0x000000a0,
	.master_mask   = BIT(0), /* Shared IRQ 0 */
};

static const struct uio_fireant_irqmux_config jaguar2_data = {
	.reg_off_ident = 0x00000094,
	.reg_off_force = 0x0000007c,
	.reg_off_clr   = 0x0000008c,
	.reg_off_set   = 0x00000090,
	.master_mask   = BIT(11),  /* SW0 */
};

static const struct uio_fireant_irqmux_config ocelot_data = {
	.reg_off_ident = 0x00000094,
	.reg_off_force = 0x0000007c,
	.reg_off_clr   = 0x0000008c,
	.reg_off_set   = 0x00000090,
	.master_mask   = BIT(10),  /* SW0 */
};

static const struct uio_fireant_irqmux_config luton_data = {
	.reg_off_ident = 0x0000009c,
	.reg_off_force = 0x00000000,
	.reg_off_clr   = 0x0000008c,
	.reg_off_set   = 0x00000090,
	.master_mask   = BIT(2),  /* SW0 */
};

static const struct of_device_id uio_of_fireant_irqmux_match[] = {
	{ .compatible = "mscc,uio_fireant_irqmux", .data = &fireant_data },
	{ .compatible = "mscc,uio_jaguar2_irqmux", .data = &jaguar2_data },
	{ .compatible = "mscc,uio_ocelot_irqmux", .data = &ocelot_data },
	{ .compatible = "mscc,uio_luton_irqmux", .data = &luton_data },
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
