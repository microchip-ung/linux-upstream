/*
 * VCoreIII interface driver for DW SPI Core
 *
 * Copyright (c) 2016, Microsemi Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

#include <linux/irq.h>

#include <asm/vcoreiii-gpio.h>
#include <linux/delay.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_vcoreiii"


// Device tree badly needed...
static int cs2gpio[] = {
	-1, -1, -1, -1,			// CS 0- 3, Controlled by SW_MODE
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2C)
	25, 26, 27, 28,			// CS 4- 7
	29, 30, 31, 32,			// CS 8-11
	33, 49, 50, 51,			// CS12-15
#elif defined(CONFIG_VTSS_VCOREIII_OCELOT)
	// Only 4 CS, dummy entries
	-1, -1, -1, -1,
#else
#error Device not supported with this wriver
#endif
};

#define NUM_CS ARRAY_SIZE(cs2gpio)

struct dw_spi_vcoreiii {
	struct dw_spi  dws;
};

static inline struct dw_spi_vcoreiii *to_dw_spi_vcoreiii(struct dw_spi *p)
{
	return p ? container_of(p, struct dw_spi_vcoreiii, dws) : NULL;
}

/*
 * Bugzilla#16259:
 *
 * The SPI master controller automatically deasserts
 * chip select when the tx fifo is empty.
 *
 */
static
void dw_spi_vcoreiii_set_cs(struct spi_device *spi, bool nEnable)
{
	bool enable = !nEnable;
	u32 cs = spi->chip_select;
	// If the CS is already configured as a GPIO the SPI framework will have dealt with it
	if (likely(spi->cs_gpio == -ENOENT)) {
		if (likely(cs < 4)) {
			/* JR2, Ferret and onwards:
			 *
			 * Use the overriding SPI boot controller since the
			 * final chip select is an OR gate between these two.
			 */
			u32 sw_mode;
			if (enable)
				sw_mode =
					VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE(1) |
					VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS(VTSS_BIT(cs));
			else
				sw_mode = VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE(1);
			writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
		} else {  // For the CS >= 4, drive as a standard GPIO
			int gpio = cs2gpio[cs];
			if (gpio >= 0) {
				vcoreiii_gpio_set_mode(gpio, VCOREIII_GPIO_MODE_NORMAL);
				vcoreiii_gpio_direction_output(gpio, !enable); // nCS driven (active-low)
			} else {
				pr_warn_ratelimited("%s: device using unsupported CS%d\n", DRIVER_NAME, cs);
			}
		}
	}
	/* Chain to base implementation */
	dw_spi_set_cs(spi, nEnable);
}

static int dw_spi_vcoreiii_probe(struct platform_device *pdev)
{
	struct dw_spi_vcoreiii *dws_vcoreiii;
	struct dw_spi *dws;
	int ret;

	//pr_info("%s driver loading\n", DRIVER_NAME);

	dws_vcoreiii = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_vcoreiii), GFP_KERNEL);
	if (!dws_vcoreiii)
		return -ENOMEM;

	// Usee SIMC
	/* IF_SI0_OWNER, select the owner of the SI interface
	 * Encoding: 0: SI Slave
	 *	     1: SI Boot Master
	 *	     2: SI Master Controller
	 */
	writel(0, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
	vcoreiii_io_mask_set(VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL,
			     VTSS_M_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL_IF_SI_OWNER,
			     VTSS_F_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL_IF_SI_OWNER(2));

	/* Set up host info */
	dws = &dws_vcoreiii->dws;
	dws->regs = VTSS_TO_SIMC;
	dws->irq = SIMC_IRQ;
	dws->bus_num = pdev->id;
	dws->max_freq = 250000000; // clk_get_rate(dws_vcoreiii->clk);
	dws->num_cs = NUM_CS;
	dws->set_cs = dw_spi_vcoreiii_set_cs;

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, dws_vcoreiii);

	return 0;

out:
	//clk_disable_unprepare(dws_vcoreiii->clk);
	return ret;
}

static int dw_spi_vcoreiii_remove(struct platform_device *pdev)
{
	struct dw_spi_vcoreiii *dws_vcoreiii = platform_get_drvdata(pdev);

	//clk_disable_unprepare(dws_vcoreiii->clk);
	dw_spi_remove_host(&dws_vcoreiii->dws);

	return 0;
}

static struct platform_driver dw_spi_vcoreiii_driver = {
	.probe		= dw_spi_vcoreiii_probe,
	.remove		= dw_spi_vcoreiii_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};
module_platform_driver(dw_spi_vcoreiii_driver);

MODULE_AUTHOR("Lars Povlsen <lars.povlsen@microsemi.com>");
MODULE_DESCRIPTION("VCoreIII interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
