/*
 * Memory-mapped interface driver for DW SPI Core
 *
 * Copyright (c) 2010, Octasic semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_mmio"

struct dw_spi_mmio {
	struct dw_spi  dws;
	struct clk     *clk;
	void           *priv;
};

#define MSCC_IF_SI_OWNER_MASK			GENMASK(1, 0)
#define MSCC_IF_SI_OWNER_SISL			0
#define MSCC_IF_SI_OWNER_SIBM			1
#define MSCC_IF_SI_OWNER_SIMC			2

#define MSCC_SPI_MST_SW_MODE			0x14

struct dw_spi_mscc_props {
	const char *syscon_name;
	u32 general_ctrl_off;
	u32 si_owner_bit;
	u32 pinctrl_bit_off;
	u32 cs_bit_off;
};

static const struct dw_spi_mscc_props dw_spi_mscc_props_ocelot = {
	.syscon_name		= "mscc,ocelot-cpu-syscon",
	.general_ctrl_off	= 0x24,
	.si_owner_bit		= 4,
	.pinctrl_bit_off	= 13,
	.cs_bit_off		= 5,
};

static const struct dw_spi_mscc_props dw_spi_mscc_props_jaguar2 = {
	.syscon_name		= "mscc,jaguar2-cpu-syscon",
	.general_ctrl_off	= 0x24,
	.si_owner_bit		= 6,
	.pinctrl_bit_off	= 13,
	.cs_bit_off		= 5,
};

static const struct dw_spi_mscc_props dw_spi_mscc_props_fireant = {
	.syscon_name		= "mscc,fireant-cpu-syscon",
	.general_ctrl_off	= 0x88,
	.si_owner_bit		= 6,
	.pinctrl_bit_off	= 22,
	.cs_bit_off		= 8,
};

struct dw_spi_mscc {
	struct regmap       		*syscon;
	void __iomem        		*spi_mst;
	const struct dw_spi_mscc_props	*props;
};

/*
 * The Designware SPI controller (referred to as master in the documentation)
 * automatically deasserts chip select when the tx fifo is empty. The chip
 * selects then needs to be either driven as GPIOs or, for the first 4 using the
 * the SPI boot controller registers. the final chip select is an OR gate
 * between the Designware SPI controller and the SPI boot controller.
 */
static void dw_spi_mscc_set_cs(struct spi_device *spi, bool enable)
{
	struct dw_spi *dws = spi_master_get_devdata(spi->master);
	struct dw_spi_mmio *dwsmmio = container_of(dws, struct dw_spi_mmio, dws);
	struct dw_spi_mscc *dwsmscc = dwsmmio->priv;
	const struct dw_spi_mscc_props *props = dwsmscc->props;
	u32 cs = spi->chip_select;

	if (cs < 4) {
		u32 sw_mode = BIT(props->pinctrl_bit_off);

		if (!enable)
			sw_mode |= (BIT(cs) << props->cs_bit_off);

		writel(sw_mode, dwsmscc->spi_mst + MSCC_SPI_MST_SW_MODE);
	}

	dw_spi_set_cs(spi, enable);
}

static int dw_spi_mscc_init(struct platform_device *pdev,
			    struct dw_spi_mmio *dwsmmio,
			    const struct dw_spi_mscc_props *props)
{
	struct dw_spi_mscc *dwsmscc;
	struct resource *res;

	dwsmscc = devm_kzalloc(&pdev->dev, sizeof(*dwsmscc), GFP_KERNEL);
	if (!dwsmscc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dwsmscc->spi_mst = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dwsmscc->spi_mst)) {
		dev_err(&pdev->dev, "SPI_MST region map failed\n");
		return PTR_ERR(dwsmscc->spi_mst);
	}

	dwsmscc->syscon = syscon_regmap_lookup_by_compatible(props->syscon_name);
	if (IS_ERR(dwsmscc->syscon))
		return PTR_ERR(dwsmscc->syscon);
	dwsmscc->props = props;

	/* Deassert all CS */
	writel(0, dwsmscc->spi_mst + MSCC_SPI_MST_SW_MODE);

	/* Select the owner of the SI interface */
	regmap_update_bits(dwsmscc->syscon, props->general_ctrl_off,
			   MSCC_IF_SI_OWNER_MASK << props->si_owner_bit,
			   MSCC_IF_SI_OWNER_SIMC << props->si_owner_bit);

	dwsmmio->dws.set_cs = dw_spi_mscc_set_cs;
	dwsmmio->priv = dwsmscc;

	return 0;
}

static int dw_spi_mmio_probe(struct platform_device *pdev)
{
	const struct dw_spi_mscc_props *props;
	struct dw_spi_mmio *dwsmmio;
	struct dw_spi *dws;
	struct resource *mem;
	int ret;
	int num_cs;

	dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mmio),
			GFP_KERNEL);
	if (!dwsmmio)
		return -ENOMEM;

	dws = &dwsmmio->dws;

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return dws->irq; /* -ENXIO */
	}

	dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmmio->clk))
		return PTR_ERR(dwsmmio->clk);
	ret = clk_prepare_enable(dwsmmio->clk);
	if (ret)
		return ret;

	dws->bus_num = pdev->id;

	dws->max_freq = clk_get_rate(dwsmmio->clk);

	device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);

	num_cs = 4;

	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);

	dws->num_cs = num_cs;

	if (pdev->dev.of_node) {
		int i;

		for (i = 0; i < dws->num_cs; i++) {
			int cs_gpio = of_get_named_gpio(pdev->dev.of_node,
					"cs-gpios", i);

			if (cs_gpio == -EPROBE_DEFER) {
				ret = cs_gpio;
				goto out;
			}

			if (gpio_is_valid(cs_gpio)) {
				ret = devm_gpio_request(&pdev->dev, cs_gpio,
						dev_name(&pdev->dev));
				if (ret)
					goto out;
			}
		}
	}

	props = device_get_match_data(&pdev->dev);
	if (props) {
		ret = dw_spi_mscc_init(pdev, dwsmmio, props);
		if (ret)
			goto out;
	}

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, dwsmmio);
	return 0;

out:
	clk_disable_unprepare(dwsmmio->clk);
	return ret;
}

static int dw_spi_mmio_remove(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio = platform_get_drvdata(pdev);

	dw_spi_remove_host(&dwsmmio->dws);
	clk_disable_unprepare(dwsmmio->clk);

	return 0;
}

static const struct of_device_id dw_spi_mmio_of_match[] = {
	{ .compatible = "snps,dw-apb-ssi", },
	{ .compatible = "mscc,ocelot-spi", .data = &dw_spi_mscc_props_ocelot},
	{ .compatible = "mscc,jaguar2-spi", .data = &dw_spi_mscc_props_jaguar2},
	{ .compatible = "mscc,fireant-spi", .data = &dw_spi_mscc_props_fireant},
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_mmio_of_match);

static struct platform_driver dw_spi_mmio_driver = {
	.probe		= dw_spi_mmio_probe,
	.remove		= dw_spi_mmio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_mmio_of_match,
	},
};
module_platform_driver(dw_spi_mmio_driver);

MODULE_AUTHOR("Jean-Hugues Deschenes <jean-hugues.deschenes@octasic.com>");
MODULE_DESCRIPTION("Memory-mapped I/O interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
