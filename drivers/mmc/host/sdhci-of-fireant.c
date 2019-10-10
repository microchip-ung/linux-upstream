// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * drivers/mmc/host/sdhci-of-fireant.c
 *
 * MCHP Fireant SoC Secure Digital Host Controller Interface.
 *
 * Copyright (c) 2019 Microchip Inc.
 *
 * Author: Lars Povlsen <lars.povlsen@microchip.com>
 */

#define DEBUG

#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/mmc/host.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>

#include "sdhci-pltfm.h"

#define ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL	(0x22 * 4)
#define MSHC_DLY_CC_MASK			GENMASK(16, 13)
#define MSHC_DLY_CC_SHIFT			13

struct sdhci_fireant_data {
	struct sdhci_host *host;
	struct regmap *cpu_ctrl;
	u32 delay_clock;
};

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void sdhci_fireant_adma_write_desc(struct sdhci_host *host, void **desc,
					  dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	pr_debug("dwcmshc_adma_write_desc: len %d\n", len);

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static void fireant_set_uhs_signaling(struct sdhci_host *host, unsigned timing)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_fireant_data *sdhci_fireant = sdhci_pltfm_priv(pltfm_host);

	pr_debug("%s: Set DLY_CC = %u, timing %u\n", mmc_hostname(host->mmc),
		 sdhci_fireant->delay_clock, timing);

	/* Ensure DLY_CC is set in HW */
	regmap_update_bits(sdhci_fireant->cpu_ctrl,
			   ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL,
			   MSHC_DLY_CC_MASK,
			   (sdhci_fireant->delay_clock << MSHC_DLY_CC_SHIFT));

	/* Call standard setup */
	sdhci_set_uhs_signaling(host, timing);
}

static const struct sdhci_ops sdhci_fireant_ops = {
	.set_clock		= sdhci_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= fireant_set_uhs_signaling,
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.reset			= sdhci_reset,
	.adma_write_desc	= sdhci_fireant_adma_write_desc,
};

static const struct sdhci_pltfm_data sdhci_fireant_pdata = {
	.quirks = 0,
	.quirks2 = SDHCI_QUIRK2_TUNING_WORK_AROUND,
	.ops = &sdhci_fireant_ops,
};

int sdhci_fireant_probe(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_fireant_data *sdhci_fireant;
	struct device_node *np = pdev->dev.of_node;

	host = sdhci_pltfm_init(pdev, &sdhci_fireant_pdata, sizeof(*sdhci_fireant));

	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	sdhci_fireant = sdhci_pltfm_priv(pltfm_host);
	sdhci_fireant->host = host;

	if (of_property_read_u32(np, "mscc,clock-delay", &sdhci_fireant->delay_clock))
		sdhci_fireant->delay_clock = 3; /* Default */

	sdhci_get_of_property(pdev);

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "parsing dt failed (%d)\n", ret);
		return ret;
	}

	sdhci_fireant->cpu_ctrl = syscon_regmap_lookup_by_compatible("mscc,fireant-cpu-syscon");
	if (IS_ERR(sdhci_fireant->cpu_ctrl)) {
		dev_err(&pdev->dev, "No CPU syscon regmap !\n");
		return PTR_ERR(sdhci_fireant->cpu_ctrl);
	}

	ret = sdhci_add_host(host);
	if (ret)
		dev_err(&pdev->dev, "sdhci_add_host() failed (%d)\n", ret);

	return ret;
}

static const struct of_device_id sdhci_fireant_of_match[] = {
	{ .compatible = "mscc,dwcmshc-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_fireant_of_match);

static struct platform_driver sdhci_fireant_driver = {
	.driver = {
		.name = "sdhci-fireant",
		.of_match_table = sdhci_fireant_of_match,
		.pm = &sdhci_pltfm_pmops,
	},
	.probe = sdhci_fireant_probe,
	.remove = sdhci_pltfm_unregister,
};

module_platform_driver(sdhci_fireant_driver);

MODULE_DESCRIPTION("Fireant SDHCI OF driver");
MODULE_AUTHOR("Lars Povlsen <lars.povlsen@microchip.com>");
MODULE_LICENSE("GPL v2");
