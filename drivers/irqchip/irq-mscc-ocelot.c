// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Microsemi Ocelot IRQ controller driver
 *
 * Copyright (c) 2017 Microsemi Corporation
 */
#include <linux/bitops.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/interrupt.h>

#define ICPU_CFG_INTR_DST_INTR_IDENT(_p,x)	(_p->reg_off_ident + 0x4 * (x))
#define ICPU_CFG_INTR_INTR_TRIGGER(_p,x)	(_p->reg_off_trigger + 0x4 * (x))

struct chip_props {
	u32 reg_off_sticky;
	u32 reg_off_ena;
	u32 reg_off_ena_clr;
	u32 reg_off_ena_set;
	u32 reg_off_ident;
	u32 reg_off_trigger;
	u32 n_irq;
};

static const struct chip_props ocelot_props = {
	.reg_off_sticky 	= 0x80 - 0x70,
	.reg_off_ena		= 0x88 - 0x70,
	.reg_off_ena_clr	= 0x8c - 0x70,
	.reg_off_ena_set	= 0x90 - 0x70,
	.reg_off_ident		= 0xa8 - 0x70,
	.reg_off_trigger	= 0xcc - 0x70,
	.n_irq			= 24,
};

static const struct chip_props serval_props = {
	.reg_off_sticky 	= 0x7c - 0x70,
	.reg_off_ena		= 0x84 - 0x70,
	.reg_off_ena_clr	= 0x88 - 0x70,
	.reg_off_ena_set	= 0x8c - 0x70,
	.reg_off_ident		= 0x90 - 0x70,
	.reg_off_trigger	= 0x74 - 0x70,
	.n_irq			= 24,
};

static const struct chip_props luton_props = {
	.reg_off_sticky 	= 0x84 - 0x84,
	.reg_off_ena		= 0x88 - 0x84,
	.reg_off_ena_clr	= 0x8c - 0x84,
	.reg_off_ena_set	= 0x90 - 0x84,
	.reg_off_ident		= 0x9c - 0x84,
	.reg_off_trigger	= 0,
	.n_irq			= 28,
};

static const struct chip_props jaguar2_props = {
	.reg_off_sticky 	= 0x80 - 0x70,
	.reg_off_ena		= 0x88 - 0x70,
	.reg_off_ena_clr	= 0x8c - 0x70,
	.reg_off_ena_set	= 0x90 - 0x70,
	.reg_off_ident		= 0xa8 - 0x70,
	.reg_off_trigger	= 0xcc - 0x70,
	.n_irq			= 28,
};

static void ocelot_irq_unmask(struct irq_data *data)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(data);
	struct irq_domain *d = data->domain;
	struct chip_props *p = d->host_data;
	struct irq_chip_type *ct = irq_data_get_chip_type(data);
	unsigned int mask = data->mask;
	u32 val;

	irq_gc_lock(gc);
	if (p->reg_off_trigger) {
		val = irq_reg_readl(gc, ICPU_CFG_INTR_INTR_TRIGGER(p, 0)) |
			irq_reg_readl(gc, ICPU_CFG_INTR_INTR_TRIGGER(p, 1));
		if (!(val & mask))
			irq_reg_writel(gc, mask, p->reg_off_sticky);
	}

	*ct->mask_cache &= ~mask;
	irq_reg_writel(gc, mask, p->reg_off_ena_set);
	irq_gc_unlock(gc);
}

static void ocelot_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_domain *d = irq_desc_get_handler_data(desc);
	struct chip_props *p = d->host_data;
	struct irq_chip_generic *gc = irq_get_domain_generic_chip(d, 0);
	u32 reg = irq_reg_readl(gc, ICPU_CFG_INTR_DST_INTR_IDENT(p, 0));

	chained_irq_enter(chip, desc);

	while (reg) {
		u32 hwirq = __fls(reg);
		generic_handle_irq(irq_find_mapping(d, hwirq));
		reg &= ~(BIT(hwirq));
	}

	chained_irq_exit(chip, desc);
}

static int __init vcoreiii_irq_init(struct device_node *node,
				    struct device_node *parent,
				    const struct chip_props *p)
{
	struct irq_domain *domain;
	struct irq_chip_generic *gc;
	int parent_irq, ret;

	pr_warn("%s: Load, %d irqs\n", node->name, p->n_irq);

	parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq)
		return -EINVAL;

	domain = irq_domain_add_linear(node, p->n_irq,
				       &irq_generic_chip_ops, NULL);
	if (!domain) {
		pr_err("%s: unable to add irq domain\n", node->name);
		return -ENOMEM;
	}

	ret = irq_alloc_domain_generic_chips(domain, p->n_irq, 1,
					     "icpu", handle_level_irq,
					     0, 0, 0);
	if (ret) {
		pr_err("%s: unable to alloc irq domain gc\n", node->name);
		goto err_domain_remove;
	}

	gc = irq_get_domain_generic_chip(domain, 0);
	gc->reg_base = of_iomap(node, 0);
	if (!gc->reg_base) {
		pr_err("%s: unable to map resource\n", node->name);
		ret = -ENOMEM;
		goto err_gc_free;
	}

	gc->chip_types[0].regs.ack = p->reg_off_sticky;
	gc->chip_types[0].regs.mask = p->reg_off_ena_clr;
	gc->chip_types[0].chip.irq_ack = irq_gc_ack_set_bit;
	gc->chip_types[0].chip.irq_mask = irq_gc_mask_set_bit;
	gc->chip_types[0].chip.irq_unmask = ocelot_irq_unmask;

	/* Mask and ack all interrupts */
	irq_reg_writel(gc, 0, p->reg_off_ena);
	irq_reg_writel(gc, 0xffffffff, p->reg_off_sticky);

	domain->host_data = (void*) p;
	irq_set_chained_handler_and_data(parent_irq, ocelot_irq_handler,
					 domain);

	return 0;

err_gc_free:
	irq_free_generic_chip(gc);

err_domain_remove:
	irq_domain_remove(domain);

	return ret;
}

static int __init ocelot_irq_init(struct device_node *node,
				  struct device_node *parent)
{
	return vcoreiii_irq_init(node, parent, &ocelot_props);
}

IRQCHIP_DECLARE(ocelot_icpu, "mscc,ocelot-icpu-intr", ocelot_irq_init);

static int __init serval_irq_init(struct device_node *node,
				  struct device_node *parent)
{
	return vcoreiii_irq_init(node, parent, &serval_props);
}

IRQCHIP_DECLARE(serval_icpu, "mscc,serval-icpu-intr", serval_irq_init);

static int __init luton_irq_init(struct device_node *node,
				 struct device_node *parent)
{
	return vcoreiii_irq_init(node, parent, &luton_props);
}

IRQCHIP_DECLARE(luton_icpu, "mscc,luton-icpu-intr", luton_irq_init);

static int __init jaguar2_irq_init(struct device_node *node,
				 struct device_node *parent)
{
	return vcoreiii_irq_init(node, parent, &jaguar2_props);
}

IRQCHIP_DECLARE(jaguar2_icpu, "mscc,jaguar2-icpu-intr", jaguar2_irq_init);
IRQCHIP_DECLARE(servalt_icpu, "mscc,servalt-icpu-intr", jaguar2_irq_init);
