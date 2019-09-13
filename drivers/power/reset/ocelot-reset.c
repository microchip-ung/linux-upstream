// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Microsemi MIPS SoC reset driver
 *
 * License: Dual MIT/GPL
 * Copyright (c) 2017 Microsemi Corporation
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

struct reset_mscc_props {
	const char *syscon_name;
	u32 protect_reg_off;
	u32 vcore_protect;
	int (*switch_core_reset)(void __iomem *base, const struct reset_mscc_props *props);
};

struct ocelot_reset_context {
	void __iomem *base;
	struct regmap *cpu_ctrl;
	const struct reset_mscc_props *props;
	struct notifier_block restart_handler;
};

#define SOFT_SWC_RST  BIT(1)
#define SOFT_CHIP_RST BIT(0)

static int ocelot_restart_handle(struct notifier_block *this,
				 unsigned long mode, void *cmd)
{
	struct ocelot_reset_context *ctx = container_of(this, struct
							ocelot_reset_context,
							restart_handler);

	/* Make sure the core is not protected from reset */
	regmap_update_bits(ctx->cpu_ctrl, ctx->props->protect_reg_off,
			   ctx->props->vcore_protect, 0);

	pr_emerg("Resetting SoC\n");

	writel(SOFT_CHIP_RST, ctx->base);

	pr_emerg("Unable to restart system\n");
	return NOTIFY_DONE;
}

static int ocelot_reset_probe(struct platform_device *pdev)
{
	struct ocelot_reset_context *ctx;
	struct resource *res;
	struct device *dev = &pdev->dev;
	int err;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->base)) {
		dev_err(dev, "No reset register!\n");
		return PTR_ERR(ctx->base);
	}

	ctx->props = device_get_match_data(dev);
	ctx->cpu_ctrl = syscon_regmap_lookup_by_compatible(ctx->props->syscon_name);
	if (IS_ERR(ctx->cpu_ctrl)) {
		dev_err(dev, "No syscon regmap named '%s'!\n", ctx->props->syscon_name);
		return PTR_ERR(ctx->cpu_ctrl);
	}

	ctx->restart_handler.notifier_call = ocelot_restart_handle;
	ctx->restart_handler.priority = 192;
	err = register_restart_handler(&ctx->restart_handler);
	if (err)
		dev_err(dev, "can't register restart notifier (err=%d)\n", err);

	return err;
}

static int fireant_switch_core_reset(void __iomem *base, const struct reset_mscc_props *props)
{
	struct regmap * cpu_ctrl;
	int timeout;

	pr_debug("fireant: Resetting Switch Core\n");

	cpu_ctrl = syscon_regmap_lookup_by_compatible(props->syscon_name);
	if (IS_ERR(cpu_ctrl)) {
		pr_err("No syscon regmap named '%s'!\n", props->syscon_name);
		return -ENXIO;
	}

	/* Make sure the core is PROTECTED from reset */
	regmap_update_bits(cpu_ctrl, props->protect_reg_off,
			   props->vcore_protect, props->vcore_protect);

	writel(SOFT_SWC_RST, base);
	for(timeout = 0; timeout < 100; timeout++) {
		if ((readl(base) & SOFT_SWC_RST) == 0) {
			pr_debug("Switch Core Reset complete.\n");
			return 0;
		}
		udelay(1);
	}

	pr_warning("Switch Core Reset timeout!\n");
	return -ENXIO;
}

static const struct reset_mscc_props reset_mscc_props_ocelot = {
	.syscon_name = "mscc,ocelot-cpu-syscon",
	.protect_reg_off = 0x20,
	.vcore_protect   = BIT(2),
};

static const struct reset_mscc_props reset_mscc_props_fireant = {
	.syscon_name = "mscc,fireant-cpu-syscon",
	.protect_reg_off = 0x84,
	.vcore_protect   = BIT(10),
	.switch_core_reset    = fireant_switch_core_reset,
};

static const struct of_device_id ocelot_reset_of_match[] = {
	{ .compatible = "mscc,ocelot-chip-reset", .data = &reset_mscc_props_ocelot },
	{ .compatible = "mscc,fireant-chip-reset", .data = &reset_mscc_props_fireant },
	{}
};

static struct platform_driver ocelot_reset_driver = {
	.probe = ocelot_reset_probe,
	.driver = {
		.name = "ocelot-chip-reset",
		.of_match_table = ocelot_reset_of_match,
	},
};
builtin_platform_driver(ocelot_reset_driver);

static int __init early_switch_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;

	np = of_find_matching_node_and_match(NULL, ocelot_reset_of_match, &match);
	if (np) {
		const struct reset_mscc_props *props = match->data;
		if (of_property_read_bool(np, "mscc,reset-switch-core") &&
		    props->switch_core_reset) {
			struct resource regs;
			void __iomem *base;
			if (of_address_to_resource(np, 0, &regs) < 0) {
				pr_err("failed to get reset registers\n");
				of_node_put(np);
				return -ENXIO;
			}
			base = ioremap_nocache(regs.start, resource_size(&regs));
			if (!base) {
				pr_err("failed to map reset registers\n");
				of_node_put(np);
				return -ENXIO;
			}

			/* Call switch reset function */
			props->switch_core_reset(base, props);

			iounmap(base);
		}

		of_node_put(np);
	}

	return 0;
}

early_initcall(early_switch_init);
