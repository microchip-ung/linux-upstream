// SPDX-License-Identifier: (GPL-2.0 OR MIT)
//
// Microsemi SoC Fireant Multifunction/PCIe driver
//
// Copyright (c) 2019 Microsemi

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/mfd/core.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/uio_driver.h>

#define DEVICE_NAME "mscc-fireant-pci"
#define PCI_VENDOR_ID_MSCC              0x101b
#define PCI_DEVICE_ID_MSCC_FIREANT      0xb006

#define FIREANT_SWITCH_BAR      0     /* Default instance in CML model */
#define FIREANT_CONFIG_BAR      2     /* amba_top instance in CML model */
#define FIREANT_SUBCPU_BAR      4     /* subcpu_sys instance in CML model */

/* CPU:INTR:INTR_STICKY */
#define CPU_INTR_STICKY_OFF                            0x100001c4
/* CPU:INTR:INTR_IDENT */
#define CPU_INTR_IDENT_OFF                             0x100001ec
/* CPU:INTR:INTR_ENA_CLR */
#define CPU_INTR_ENA_CLR_OFF                           0x100001dc
/* CPU:INTR:INTR_ENA_SET */
#define CPU_INTR_ENA_SET_OFF                           0x100001e4
/* CPU:INTR:DST_INTR_MAP[2] */
#define CPU_INTR_DST_MAP_R_OFF(ridx)                   (0x100001f4 + (ridx*4))
/* CPU:INTR:INTR_TRIGGER[2] */
#define CPU_INTR_TRIGGER_R_OFF(ridx)                   (0x100001ac + (ridx*4))
/* CPU:PCIE:PCIE_INTR_COMMON_CFG[2] */
#define CPU_PCIE_INTR_COMMON_CFG_R_OFF(ridx)           (0x1000018c + (ridx*4))
/* DEVCPU_GCB:CHIP_REGS:CHIP_ID */
#define DEVCPU_GCB_CHIP_REGS_ID_OFF                    0x01010000

/* CPU:PCIE:PCIE_INTR_COMMON_CFG[2] */
#define CPU_PCIE_INTR_COMMON_CFG_ENA                   BIT(1)

#define GET_REGION(off)                                ((off) & GENMASK(31, 28))
#define GET_REGION_INDEX(off)                          ((off) >> 28)
#define GET_ADDRESS(off)                               ((off) & GENMASK(27, 0))

static struct pci_device_id fireant_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_MSCC, PCI_DEVICE_ID_MSCC_FIREANT) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, fireant_ids);

struct uio_fireant {
	struct uio_info uio;
	spinlock_t lock;
	unsigned long flags;
	struct pci_dev *pdev;
};

static struct resource fireant_io_resources[] = {
	{
		.flags = IORESOURCE_MEM,
	},
	{
		.flags = IORESOURCE_MEM,
	},
};

static const struct mfd_cell fireant_mfd_cells[] = {
	{
		.name = "fireant_fdma",
		.num_resources = ARRAY_SIZE(fireant_io_resources),
		.resources = fireant_io_resources,
		.of_compatible = "mscc,vsc7568-fdma",
	},
};

static u32 fireant_pci_readl(struct uio_fireant *priv, u32 reg)
{
	void __iomem *addr = 
		priv->uio.mem[GET_REGION_INDEX(reg)].internal_addr + 
		GET_ADDRESS(reg);
	u32 data = readl(addr);
#ifdef FDMA_LOG
	pr_debug("%s:%d %s: addr 0x%llx, data 0x%08x\n", 
		__FILE__, __LINE__, __func__,
		priv->uio.mem[GET_REGION_INDEX(reg)].addr + 
		GET_ADDRESS(reg),
		data);
#endif
	return data;
}

static void fireant_pci_writel(struct uio_fireant *priv, u32 reg, u32 data)
{
	void __iomem *addr = 
		priv->uio.mem[GET_REGION_INDEX(reg)].internal_addr + 
		GET_ADDRESS(reg);
#ifdef FDMA_LOG
	pr_debug("%s:%d %s: addr 0x%llx, data 0x%08x\n", 
		__FILE__, __LINE__, __func__, 
		priv->uio.mem[GET_REGION_INDEX(reg)].addr + 
			GET_ADDRESS(reg),
		data);
#endif
	writel(data, addr);
}

static int fireant_pci_irqcontrol(struct uio_info *info, s32 irq_on)
{
	struct uio_fireant *priv = info->priv;
	unsigned long flags;

	pr_debug("%s:%d %s: irq_on %d\n", 
		__FILE__, __LINE__, __func__, irq_on);
	spin_lock_irqsave(&priv->lock, flags);
	if (irq_on) {
		if (test_and_clear_bit(0, &priv->flags)) {
			pci_intx(priv->pdev, 1);
		}
	} else {
		if (!test_and_set_bit(0, &priv->flags)) {
			pci_intx(priv->pdev, 0);
		}
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void fireant_pci_irq_unmask(struct irq_data *data)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(data);
	struct irq_chip_type *ct = irq_data_get_chip_type(data);
	unsigned int mask = data->mask;
	u32 val;

	pr_debug("%s:%d %s: irq %d\n", 
		__FILE__, __LINE__, __func__, data->irq);
	irq_gc_lock(gc);
	val = irq_reg_readl(gc, GET_ADDRESS(CPU_INTR_TRIGGER_R_OFF(0))) |
	      irq_reg_readl(gc, GET_ADDRESS(CPU_INTR_TRIGGER_R_OFF(1)));
	if (!(val & mask)) {
		irq_reg_writel(gc, mask, GET_ADDRESS(CPU_INTR_STICKY_OFF));
	}

	*ct->mask_cache &= ~mask;
	irq_reg_writel(gc, mask, GET_ADDRESS(CPU_INTR_ENA_SET_OFF));
	irq_gc_unlock(gc);
}

static void fireant_pci_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_domain *d = irq_desc_get_handler_data(desc);
	struct irq_chip_generic *gc = irq_get_domain_generic_chip(d, 0);
	u32 uio, reg = irq_reg_readl(gc, GET_ADDRESS(CPU_INTR_IDENT_OFF));
	u32 mask = *gc->chip_types[0].mask_cache;
	struct uio_fireant *priv = gc->private;

	uio = reg & mask;
	reg &= ~mask;

	pr_debug("%s:%d %s: irq %d: ident: 0x%x\n", 
		__FILE__, __LINE__, __func__, desc->irq_data.irq, reg);
	chained_irq_enter(chip, desc);

	while (reg) {
		u32 hwirq = __fls(reg);

		pr_debug("%s:%d %s: hwirq: %u maps to IRQ: %d\n", 
			__FILE__, __LINE__, __func__, 
			hwirq,
			irq_find_mapping(d, hwirq));
		generic_handle_irq(irq_find_mapping(d, hwirq));
		reg &= ~(BIT(hwirq));
	}

	chained_irq_exit(chip, desc);

	if (uio) {
		if (!test_and_set_bit(0, &priv->flags))
			pci_intx(priv->pdev, 0);
		uio_event_notify(&priv->uio);
	}

}

static int __init fireant_pci_irq_common_init(struct uio_fireant *priv,
	struct device_node *node,
	struct pci_dev *pdev,
	int size)
{
	struct irq_domain *domain;
	struct irq_chip_generic *gc;
	int ret;

	pr_debug("%s:%d %s: Using IRQ: %d\n", 
		__FILE__, __LINE__, __func__, pdev->irq);
	if (!pdev->irq || !node) {
		pr_err("no node or IRQ: %d, 0x%px\n", pdev->irq, node);
		return -EINVAL;
	}

	/* TODO: The current implementation only handles the first 32 interrupts
	 * and there are 50 interrupts from Fireant in all
	 */
	domain = irq_domain_add_linear(node, size, &irq_generic_chip_ops, NULL);
	if (!domain) {
		pr_err("%s: unable to add irq domain\n", node->name);
		return -ENOMEM;
	}

	ret = irq_alloc_domain_generic_chips(domain, size, 1, "cpu",
					     handle_level_irq, 0, 0, 0);
	if (ret) {
		pr_err("%s: unable to alloc irq domain gc\n", node->name);
		goto err_domain_remove;
	}

	gc = irq_get_domain_generic_chip(domain, 0);
	gc->reg_base = priv->uio.mem[1].internal_addr;
	if (!gc->reg_base) {
		pr_err("%s: unable to map resource\n", node->name);
		ret = -ENOMEM;
		goto err_gc_free;
	}

	gc->chip_types[0].regs.ack = GET_ADDRESS(CPU_INTR_STICKY_OFF);
	gc->chip_types[0].regs.mask = GET_ADDRESS(CPU_INTR_ENA_CLR_OFF);
	gc->chip_types[0].chip.irq_ack = irq_gc_ack_set_bit;
	gc->chip_types[0].chip.irq_mask = irq_gc_mask_set_bit;
	gc->chip_types[0].chip.irq_unmask = fireant_pci_irq_unmask;
	gc->mask_cache = 0xffffffff;

	gc->private = priv;

	/* Mask and ack all interrupts */
	irq_reg_writel(gc, 0, GET_ADDRESS(CPU_INTR_ENA_SET_OFF));
	irq_reg_writel(gc, ~0, GET_ADDRESS(CPU_INTR_STICKY_OFF));

	pr_debug("%s:%d %s: Chaining IRQ: %d\n", 
		__FILE__, __LINE__, __func__, pdev->irq);
	irq_set_chained_handler_and_data(pdev->irq, fireant_pci_irq_handler,
		domain);

	return 0;

err_gc_free:
	irq_free_generic_chip(gc);

err_domain_remove:
	irq_domain_remove(domain);

	return ret;
}

static int fireant_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device_node *devnode;
	struct uio_fireant *priv;
	struct uio_info *info;
	u32 chip_id;
	int ret;

	pr_debug("%s:%d %s: probe 0x%04x:0x%04x 0x%04x:0x%08x C:0x%08x F:0x%08x\n", 
		__FILE__, __LINE__, __func__, 
		pdev->vendor, pdev->device,
		pdev->subsystem_vendor, pdev->subsystem_device,
		pdev->class, pdev->devfn);
	devnode = pdev->dev.of_node;
	if (!devnode) {
		pr_err("%s:%d %s: no platform device node\n", 
			__FILE__, __LINE__, __func__);
		devnode = of_find_compatible_node(NULL, NULL, "pci101b,b006");
		if (!devnode) {
			return -ENODEV;
		}
		pdev->dev.of_node = devnode;
	}

	/* TODO: Remove this check when the PCI configuration is correct */
	if (pdev->devfn > 0) {
		pr_err("%s:%d %s: Not accepting function 1 or higher\n",
			__FILE__, __LINE__, __func__);
		return -ENOENT;
	}
	pr_debug("%s:%d %s: OK\n", __FILE__, __LINE__, __func__);

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Could not enable PCI device\n");
		return ret;
	}
	pci_set_master(pdev);
	dev_info(&pdev->dev, "Device is master\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		pr_err("%s:%d %s: No memory\n",
			__FILE__, __LINE__, __func__);
		return -ENOMEM;
	}

	info = &priv->uio;
	info->priv = priv;


	info->mem[0].addr = pci_resource_start(pdev, FIREANT_SWITCH_BAR);
	info->mem[0].size = pci_resource_len(pdev, FIREANT_SWITCH_BAR);
	info->mem[0].memtype = UIO_MEM_PHYS;
	info->mem[0].internal_addr = pcim_iomap(pdev, FIREANT_SWITCH_BAR, 0);

	info->mem[1].addr = pci_resource_start(pdev, FIREANT_CONFIG_BAR);
	info->mem[1].size = pci_resource_len(pdev, FIREANT_CONFIG_BAR);
	info->mem[1].memtype = UIO_MEM_PHYS;
	info->mem[1].internal_addr = pcim_iomap(pdev, FIREANT_CONFIG_BAR, 0);

	if (!info->mem[0].internal_addr || !info->mem[1].internal_addr) {
		pr_err("%s:%d %s: Could not map PCI bars\n",
			__FILE__, __LINE__, __func__);
		return -ENOMEM;
	}

	chip_id = fireant_pci_readl(priv, DEVCPU_GCB_CHIP_REGS_ID_OFF);
	pr_debug("%s:%d %s: Fireant: %x\n",
		__FILE__, __LINE__, __func__, chip_id);
	if (chip_id != 0x07568445) {
		pr_err("%s:%d %s: Chip ID error\n",
			__FILE__, __LINE__, __func__);
		return -ENOMEM;
	}

	info->name = "sparx5_switch";
	info->version = "0";
	info->irq = UIO_IRQ_CUSTOM;
	info->irqcontrol = fireant_pci_irqcontrol;

	spin_lock_init(&priv->lock);
	priv->flags = 0; /* interrupt is enabled to begin with */
	priv->pdev = pdev;

	ret = uio_register_device(&pdev->dev, info);
	if (ret) {
		pr_err("%s:%d %s: Could not register UIO driver: %d\n",
			__FILE__, __LINE__, __func__, ret);
		return ret;
	}
	pci_set_drvdata(pdev, info);

	pr_info("%s:%d %s: FireAnt UIO Driver with PCI IRQ: legacy: %d\n",
		__FILE__, __LINE__, __func__, pdev->irq);

	/* Map interrupts to destination EXT_DST0 (0) */
	fireant_pci_writel(priv, CPU_INTR_DST_MAP_R_OFF(0), ~0);
	/* Set Level activated interrupts */
	fireant_pci_writel(priv, CPU_INTR_TRIGGER_R_OFF(0), 0);
	fireant_pci_writel(priv, CPU_INTR_TRIGGER_R_OFF(1), 0);
	/* Enable PCIe Legacy interrupt on Function 0 using EXT_DST0 */
	fireant_pci_writel(priv, CPU_PCIE_INTR_COMMON_CFG_R_OFF(0), CPU_PCIE_INTR_COMMON_CFG_ENA);

	if (fireant_pci_irq_common_init(priv, devnode, pdev, 32)) {
		goto out_unregister;
	}
	fireant_io_resources[0].start = info->mem[0].addr;
	fireant_io_resources[0].end = info->mem[0].addr + info->mem[0].size - 1;
	fireant_io_resources[1].start = info->mem[1].addr;
	fireant_io_resources[1].end = info->mem[1].addr + info->mem[1].size - 1;
	return devm_mfd_add_devices(&pdev->dev, -1, fireant_mfd_cells, 
		ARRAY_SIZE(fireant_mfd_cells), NULL, 0, NULL);

out_unregister:
	pr_err("%s:%d %s: unregister UIO driver\n",
		__FILE__, __LINE__, __func__);
	uio_unregister_device(info);
	return -ENODEV;
}

static void fireant_pci_remove(struct pci_dev *pdev)
{
	pr_debug("%s:%d %s: done\n", __FILE__, __LINE__, __func__);
}

static struct pci_driver fireant_pci_driver = {
	.name = DEVICE_NAME,
	.id_table = fireant_ids,
	.probe = fireant_pci_probe,
	.remove = fireant_pci_remove,
};


module_pci_driver(fireant_pci_driver);

MODULE_AUTHOR("Steen Hegelund <steen.hegelund@microchip.com>");
MODULE_DESCRIPTION("Microsemi FireAnt PCI driver");
MODULE_LICENSE("Dual MIT/GPL");
