/*
 * Memory-mapped interface driver for SPI master controller on Jaguar2
 *
 * Copyright (c) 2014, Wenxi Jin.
 *
 * based on: spi-dw-mmio.c
 *  Copyright (c) 2010, Octasic semiconductor.
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
#include <linux/spi/spi_jaguar2.h>
#include <linux/scatterlist.h>
#include <linux/module.h>

#include "spi_jaguar2.h"

struct jaguar2_spi_mmio {
	struct jaguar2_spi  jaguar2s;
	struct clk     *clk;
};

static int jaguar2_spi_mmio_probe(struct platform_device *pdev)
{
	struct jaguar2_spi_mmio *jaguar2smmio;
	struct jaguar2_spi *jaguar2s;
	int ret;

	jaguar2smmio = devm_kzalloc(&pdev->dev, sizeof(struct jaguar2_spi_mmio),
			GFP_KERNEL);
	if (!jaguar2smmio)
		return -ENOMEM;

	jaguar2s = &jaguar2smmio->jaguar2s;

 	jaguar2s->irq = SIMC_IRQ;

	jaguar2smmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(jaguar2smmio->clk))
            return PTR_ERR(jaguar2smmio->clk);
        ret = clk_prepare_enable(jaguar2smmio->clk);
	if (ret)
            return ret;

	jaguar2s->bus_num = 0;
	jaguar2s->num_cs = 4;
	jaguar2s->max_freq = clk_get_rate(jaguar2smmio->clk);

	ret = jaguar2_spi_add_host(&pdev->dev, jaguar2s);
	if (ret)
            goto out;

	platform_set_drvdata(pdev, jaguar2smmio);
	return 0;

out:
	clk_disable_unprepare(jaguar2smmio->clk);
	return ret;
}

static int jaguar2_spi_mmio_remove(struct platform_device *pdev)
{
	struct jaguar2_spi_mmio *jaguar2smmio = platform_get_drvdata(pdev);

	clk_disable_unprepare(jaguar2smmio->clk);
	jaguar2_spi_remove_host(&jaguar2smmio->jaguar2s);

	return 0;
}

static struct platform_driver jaguar2_spi_mmio_driver = {
	.probe		= jaguar2_spi_mmio_probe,
	.remove		= jaguar2_spi_mmio_remove,
	.driver		= {
		.name	= SPI_VCOREIII_PLATDEV_NAME,
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(jaguar2_spi_mmio_driver);

MODULE_AUTHOR("Wenxi Jin <wjin@vitesse.com>");
MODULE_DESCRIPTION("Memory-mapped I/O interface driver for Jaguar2 SPI master controller");
MODULE_LICENSE("GPL v2");
