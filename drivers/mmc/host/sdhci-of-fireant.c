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

//#define DEBUG
//#define TRACE_REGISTER

#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>
#include <linux/dma-mapping.h>

#include "sdhci-pltfm.h"

#define CPU_REGS_GENERAL_CTRL	(0x22 * 4)
#define  MSHC_DLY_CC_MASK	GENMASK(16, 13)
#define  MSHC_DLY_CC_SHIFT	13
#define  MSHC_DLY_CC_MAX	15

#define CPU_REGS_PROC_CTRL	(0x2C * 4)
#define  ACP_CACHE_FORCE_ENA	BIT(4)
#define  ACP_AWCACHE		BIT(3)
#define  ACP_ARCACHE		BIT(2)
#define  ACP_CACHE_MASK		(ACP_CACHE_FORCE_ENA|ACP_AWCACHE|ACP_ARCACHE)

#define MSHC2_VERSION			0x500	/* Off 0x140, reg 0x0 */
#define MSHC2_TYPE			0x504	/* Off 0x140, reg 0x1 */
#define MSHC2_EMMC_CTRL			0x52c	/* Off 0x140, reg 0xB */
#define  MSHC2_EMMC_CTRL_EMMC_RST_N	BIT(2)
#define  MSHC2_EMMC_CTRL_IS_EMMC	BIT(0)

struct sdhci_fireant_data {
	struct sdhci_host *host;
	struct regmap *cpu_ctrl;
	int delay_clock;
	struct device_attribute dev_delay_clock;
};

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

#if defined(TRACE_REGISTER)
static void sdhci_fa_writel(struct sdhci_host *host, u32 val, int reg)
{
	pr_debug("$$$ writel(0x%08x, 0x%02x)\n", val, reg);
	writel(val, host->ioaddr + reg);
}

static void sdhci_fa_writew(struct sdhci_host *host, u16 val, int reg)
{
	pr_debug("$$$ writew(0x%04x, 0x%02x)\n", val, reg);
	writew(val, host->ioaddr + reg);
}

static void sdhci_fa_writeb(struct sdhci_host *host, u8 val, int reg)
{
	pr_debug("$$$ writeb(0x%02x, 0x%02x)\n", val, reg);
	writeb(val, host->ioaddr + reg);
}
#endif

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void sdhci_fireant_adma_write_desc(struct sdhci_host *host, void **desc,
					  dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	pr_debug("write_desc: cmd %02x: len %d, offset 0x%0llx\n",
		 cmd, len, addr);

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	pr_debug("write_desc: splitting dma len %d, offset 0x%0llx\n",
		 len, addr);

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static void fireant_set_cacheable(struct sdhci_host *host, u32 value)
				  {
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_fireant_data *sdhci_fireant = sdhci_pltfm_priv(pltfm_host);

	pr_debug("%s: Set Cacheable = 0x%x\n", mmc_hostname(host->mmc), value);

	/* Update ACP caching attributes in HW */
	regmap_update_bits(sdhci_fireant->cpu_ctrl,
			   CPU_REGS_PROC_CTRL, ACP_CACHE_MASK, value);
}

static void fireant_set_delay(struct sdhci_host *host, u8 value)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_fireant_data *sdhci_fireant = sdhci_pltfm_priv(pltfm_host);

	pr_debug("%s: Set DLY_CC = %u\n", mmc_hostname(host->mmc), value);

	/* Update DLY_CC in HW */
	regmap_update_bits(sdhci_fireant->cpu_ctrl,
			   CPU_REGS_GENERAL_CTRL,
			   MSHC_DLY_CC_MASK,
			   (value << MSHC_DLY_CC_SHIFT));
}

static void sdhci_fireant_set_emmc(struct sdhci_host *host)
{
	if (!mmc_card_is_removable(host->mmc)) {
		u8 value;
		value = sdhci_readb(host, MSHC2_EMMC_CTRL);
		if (!(value & MSHC2_EMMC_CTRL_IS_EMMC)) {
			pr_debug("Get EMMC_CTRL: 0x%08x\n", value);
			value |= MSHC2_EMMC_CTRL_IS_EMMC;
			pr_debug("Set EMMC_CTRL: 0x%08x\n", value);
			sdhci_writeb(host, value, MSHC2_EMMC_CTRL);
		}
	}
}

static void sdhci_fireant_reset_emmc(struct sdhci_host *host)
{
	u8 value;
	pr_debug("Toggle EMMC_CTRL.EMMC_RST_N\n");
	value = sdhci_readb(host, MSHC2_EMMC_CTRL) &
		~MSHC2_EMMC_CTRL_EMMC_RST_N;
	sdhci_writeb(host, value, MSHC2_EMMC_CTRL);
	/* For eMMC, minimum is 1us but give it 10us for good measure */
	udelay(10);
	sdhci_writeb(host, value | MSHC2_EMMC_CTRL_EMMC_RST_N,
		     MSHC2_EMMC_CTRL);
	/* For eMMC, minimum is 200us but give it 300us for good measure */
	udelay(300);
}

static void sdhci_fireant_reset(struct sdhci_host *host, u8 mask)
{
	pr_debug("*** RESET: mask %d\n", mask);

	sdhci_reset(host, mask);

	/* Be sure CARD_IS_EMMC stays set */
	sdhci_fireant_set_emmc(host);
}

static const struct sdhci_ops sdhci_fireant_ops = {
#if defined(TRACE_REGISTER)
	.write_l		= sdhci_fa_writel,
	.write_w		= sdhci_fa_writew,
	.write_b		= sdhci_fa_writeb,
#endif
	.set_clock		= sdhci_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.reset			= sdhci_fireant_reset,
	.adma_write_desc	= sdhci_fireant_adma_write_desc,
};

static const struct sdhci_pltfm_data sdhci_fireant_pdata = {
	.quirks  = 0,
	.quirks2 = SDHCI_QUIRK2_NO_1_8_V, /* No sdr104, ddr50, etc */
	.ops = &sdhci_fireant_ops,
};

static ssize_t fireant_delay_clock_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct sdhci_fireant_data *sdhci_fireant = container_of(attr,
								struct sdhci_fireant_data,
								dev_delay_clock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", sdhci_fireant->delay_clock);
}

static ssize_t fireant_delay_clock_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int delay_clock;
	struct sdhci_fireant_data *sdhci_fireant = container_of(attr,
								struct sdhci_fireant_data,
								dev_delay_clock);

	if (sscanf(buf, "%u", &delay_clock) != 1 ||
	    delay_clock > MSHC_DLY_CC_MAX) {
		printk(KERN_ERR "sdhci-of-fireant: wrong parameter format.\n");
		return -EINVAL;
	}

	sdhci_fireant->delay_clock = delay_clock;
	fireant_set_delay(sdhci_fireant->host, sdhci_fireant->delay_clock);

	return strlen(buf);
}

int sdhci_fireant_probe(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_fireant_data *sdhci_fireant;
	struct device_node *np = pdev->dev.of_node;
	u32 value;
	u32 extra;

	host = sdhci_pltfm_init(pdev, &sdhci_fireant_pdata, sizeof(*sdhci_fireant));

	if (IS_ERR(host))
		return PTR_ERR(host);

	/*
	 * extra adma table cnt for cross 128M boundary handling.
	 */
	extra = DIV_ROUND_UP_ULL(dma_get_required_mask(&pdev->dev), SZ_128M);
	if (extra > SDHCI_MAX_SEGS)
		extra = SDHCI_MAX_SEGS;
	host->adma_table_cnt += extra;

	pltfm_host = sdhci_priv(host);
	sdhci_fireant = sdhci_pltfm_priv(pltfm_host);
	sdhci_fireant->host = host;

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		ret = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", ret);
		goto free_pltfm;
	}
	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret)
		goto free_pltfm;

	if (!of_property_read_u32(np, "mscc,clock-delay", &value) &&
	    value <= MSHC_DLY_CC_MAX)
		sdhci_fireant->delay_clock = value;
	else
		sdhci_fireant->delay_clock = -1; /* Autotune */

	/* Sysfs delay_clock interface */
	sdhci_fireant->dev_delay_clock.show = fireant_delay_clock_show;
	sdhci_fireant->dev_delay_clock.store = fireant_delay_clock_store;
	sysfs_attr_init(&sdhci_fireant->dev_delay_clock.attr);
	sdhci_fireant->dev_delay_clock.attr.name = "delay_clock";
	sdhci_fireant->dev_delay_clock.attr.mode = S_IRUGO | S_IWUSR;
	ret = device_create_file(&pdev->dev, &sdhci_fireant->dev_delay_clock);
	if (ret)
		dev_err(&pdev->dev, "failure creating '%s' device file",
			sdhci_fireant->dev_delay_clock.attr.name);

	sdhci_get_of_property(pdev);

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_clk;

	sdhci_fireant->cpu_ctrl = syscon_regmap_lookup_by_compatible("mscc,fireant-cpu-syscon");
	if (IS_ERR(sdhci_fireant->cpu_ctrl)) {
		dev_err(&pdev->dev, "No CPU syscon regmap !\n");
		ret = PTR_ERR(sdhci_fireant->cpu_ctrl);
		goto err_clk;
	}

	if (sdhci_fireant->delay_clock >= 0)
		fireant_set_delay(host, sdhci_fireant->delay_clock);

	if (!mmc_card_is_removable(host->mmc)) {
		/* Do a HW reset of eMMC card */
		sdhci_fireant_reset_emmc(host);
		/* Update EMMC_CTRL */
		sdhci_fireant_set_emmc(host);
		/* If eMMC, disable SD and SDIO */
		host->mmc->caps2 |= (MMC_CAP2_NO_SDIO|MMC_CAP2_NO_SD);
	}

	/* Enable v4 mode */
	//sdhci_enable_v4_mode(host);

	ret = sdhci_add_host(host);
	if (ret)
		dev_err(&pdev->dev, "sdhci_add_host() failed (%d)\n", ret);

	/* Set AXI bus master to use un-cached access (for DMA) */
	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA) &&
	    IS_ENABLED(CONFIG_DMA_DECLARE_COHERENT))
		fireant_set_cacheable(host, ACP_CACHE_FORCE_ENA);

	pr_debug("SDHC version: 0x%08x\n", sdhci_readl(host, MSHC2_VERSION));
	pr_debug("SDHC type:    0x%08x\n", sdhci_readl(host, MSHC2_TYPE));

	return ret;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
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
