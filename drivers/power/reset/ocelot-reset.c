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

struct reset_props {
	const char *syscon;
	u32 protect_reg;
	u32 vcore_protect;
	u32 if_si_owner_bit;
};

struct ocelot_reset_context {
	void __iomem *base;
	struct regmap *cpu_ctrl;
	const struct reset_props *props;
	struct notifier_block restart_handler;
};

#define SOFT_SWC_RST  BIT(1)
#define SOFT_CHIP_RST BIT(0)

#define ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL	0x24
#define IF_SI_OWNER_MASK			GENMASK(1, 0)
#define IF_SI_OWNER_SISL			0
#define IF_SI_OWNER_SIBM			1
#define IF_SI_OWNER_SIMC			2

static int ocelot_switch_core_reset(const struct ocelot_reset_context *ctx)
{

	const char *driver = "ocelot-reset";
	int timeout;

	pr_notice("%s: Resetting Switch Core\n", driver);

	/* Make sure the core is PROTECTED from reset */
	regmap_update_bits(ctx->cpu_ctrl, ctx->props->protect_reg,
			   ctx->props->vcore_protect,
			   ctx->props->vcore_protect);

	writel(SOFT_SWC_RST, ctx->base);
	for (timeout = 0; timeout < 100; timeout++) {
		if ((readl(ctx->base) & SOFT_SWC_RST) == 0) {
			pr_debug("%s: Switch Core Reset complete.\n", driver);
			return 0;
		}
		udelay(1);
	}

	pr_warn("%s: Switch Core Reset timeout!\n", driver);
	return -ENXIO;
}

static int ocelot_restart_handle(struct notifier_block *this,
				 unsigned long mode, void *cmd)
{
	struct ocelot_reset_context *ctx = container_of(this, struct
							ocelot_reset_context,
							restart_handler);
	u32 if_si_owner_bit = ctx->props->if_si_owner_bit;

	/* Make sure the core is not protected from reset */
	regmap_update_bits(ctx->cpu_ctrl, ctx->props->protect_reg,
			   ctx->props->vcore_protect, 0);

	/* Make the SI back to boot mode */
	regmap_update_bits(ctx->cpu_ctrl, ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL,
			   IF_SI_OWNER_MASK << if_si_owner_bit,
			   IF_SI_OWNER_SIBM << if_si_owner_bit);

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
	if (IS_ERR(ctx->base))
		return PTR_ERR(ctx->base);

	ctx->props = device_get_match_data(dev);

	ctx->cpu_ctrl = syscon_regmap_lookup_by_compatible(ctx->props->syscon);
	if (IS_ERR(ctx->cpu_ctrl)) {
		dev_err(dev, "No syscon map: %s\n", ctx->props->syscon);
		return PTR_ERR(ctx->cpu_ctrl);
	}

	/* Optionally, call switch reset function */
	if (of_property_read_bool(pdev->dev.of_node,
				  "microchip,reset-switch-core"))
		ocelot_switch_core_reset(ctx);

	ctx->restart_handler.notifier_call = ocelot_restart_handle;
	ctx->restart_handler.priority = 192;
	err = register_restart_handler(&ctx->restart_handler);
	if (err)
		dev_err(dev, "can't register restart notifier (err=%d)\n", err);

	return err;
}

static const struct reset_props reset_props_ocelot = {
	.syscon		 = "mscc,ocelot-cpu-syscon",
	.protect_reg     = 0x20,
	.vcore_protect   = BIT(2),
	.if_si_owner_bit = 4,
};

static const struct reset_props reset_props_sparx5 = {
	.syscon		 = "microchip,sparx5-cpu-syscon",
	.protect_reg     = 0x84,
	.vcore_protect   = BIT(10),
	.if_si_owner_bit = 6,
};

static const struct of_device_id ocelot_reset_of_match[] = {
	{
		.compatible = "mscc,ocelot-chip-reset",
		.data = &reset_props_ocelot
	}, {
		.compatible = "microchip,sparx5-chip-reset",
		.data = &reset_props_sparx5
	},
	{ /*sentinel*/ }
};

static struct platform_driver ocelot_reset_driver = {
	.probe = ocelot_reset_probe,
	.driver = {
		.name = "ocelot-chip-reset",
		.of_match_table = ocelot_reset_of_match,
	},
};

static int __init reset_init(void)
{
	return platform_driver_register(&ocelot_reset_driver);
}
postcore_initcall(reset_init);
