/* Copyright (c) 2015 Microsemi Corporation

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

/*
 * Bitbanging SPI bus driver for VCore-III / Serval1 / Jaguar2
 *
 * Copyright (c) 2011 Lars Povlsen
 *
 * based on spi_gpio.c
 *  Copyright (c) 2008 Piotr Skamruk
 *  Copyright (c) 2008 Michael Buesch
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spi_vcoreiii.h>

#include <asm/vcoreiii.h>
#include <asm/vcoreiii-gpio.h>

// On JR2, VTSS_F_xxx() macros for single-bit-fields have been
// replaced by VTSS_M_xxx() macros. The VTSS_F_xxx() macros
// all take a parameter.
#if !defined(VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE)
#define VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE
#endif
#if !defined(VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK_OE)
#define VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK_OE VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK_OE
#endif
#if !defined(VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK)
#define VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK
#endif
#if !defined(VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO_OE)
#define VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO_OE VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO_OE
#endif
#if !defined(VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO)
#define VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO
#endif
#if !defined(VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDI)
#define VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDI VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDI
#endif

#include <asm/atomic.h>

struct spi_vcoreiii {
	struct spi_bitbang bitbang;
	struct spi_vcoreiii_platform_data *info;
	u32    bb_cur;
};


static inline struct spi_vcoreiii *spidev_to_sg(struct spi_device *dev)
{
	return spi_master_get_devdata(dev->master);
}

static inline void setsck(struct spi_device *dev, int val)
{
	struct spi_vcoreiii *sp = spidev_to_sg(dev);
	if(val)
		sp->bb_cur |= VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK;
	else
		sp->bb_cur &= ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK;
	writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
}

static inline void setmosi(struct spi_device *dev, int val)
{
	struct spi_vcoreiii *sp = spidev_to_sg(dev);
	sp->bb_cur |= VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO_OE;
	if(val)
		sp->bb_cur |= VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO;
	else
		sp->bb_cur &= ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDO;
	writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
}

static inline u32 getmiso(struct spi_device *dev)
{
	return (readl(VTSS_ICPU_CFG_SPI_MST_SW_MODE) & VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SDI) ? 1 : 0;
}

#define spidelay(nsecs)	ndelay(nsecs)

#include "spi-bitbang-txrx.h"

static u32 spi_vcoreiii_txrx_mode0(struct spi_device *spi,
			       unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, 0, word, bits);
}

static void spi_vcoreiii_chipselect_set_gpio_cs(struct spi_device *dev, int on, int gpio)
{
    int cs_high = !!(dev->mode & SPI_CS_HIGH);
    if (on) {
        vcoreiii_gpio_direction_output(gpio, cs_high);
    } else {
        vcoreiii_gpio_set_value(gpio, !cs_high);
    }
}

static void spi_vcoreiii_chipselect_set_gpio_lo(struct spi_device *dev, u32 mask, u32 value)
{
    if (mask) {
        writel(mask | readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OE), VTSS_DEVCPU_GCB_GPIO_GPIO_OE);
        writel(mask, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_CLR);
        writel(value, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_SET);
    }
}

#if defined(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1)
static void spi_vcoreiii_chipselect_set_gpio_hi(struct spi_device *dev, u32 mask, u32 value)
{
    if (mask) {
        writel(mask | readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1), VTSS_DEVCPU_GCB_GPIO_GPIO_OE1);
        writel(mask, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_CLR1);
        writel(value, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_SET1);
    }
}
#endif

static void spi_vcoreiii_chipselect_mux_setcs(struct spi_device *dev, int on)
{
    if (dev->chip_select < SPI_VCOREIII_NUM_HW_CS) {
        // Hw nCS
        u32 mode = readl(VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        if (on) {
            // Set CS, mask
            writel(mode |
                   VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS_OE(VTSS_BIT(dev->chip_select)) | /* CS_OE */
                   VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS(VTSS_BIT(dev->chip_select)),
                   VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        } else {
            // Drop CS
            writel(mode & ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS,
                   VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        }
    } else {
        // Straightforward GPIO CS
        spi_vcoreiii_chipselect_set_gpio_cs(dev, on, dev->chip_select - SPI_VCOREIII_NUM_HW_CS);
    }
}

static void spi_vcoreiii_chipselect_mux(struct spi_device *dev, int on)
{
    struct spi_vcoreiii_controller_data *sd = dev->controller_data;

    if (on) {
        // Set MUX value
        spi_vcoreiii_chipselect_set_gpio_lo(dev, sd->mask_lo, sd->value_lo);
#if defined(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1)
        spi_vcoreiii_chipselect_set_gpio_hi(dev, sd->mask_hi, sd->value_hi);
#endif
    }
    // Now drive actual CS
    spi_vcoreiii_chipselect_mux_setcs(dev, on);
}

static void spi_vcoreiii_chipselect_gpio(struct spi_device *dev, int on)
{
    u32 gpio = dev->chip_select - SPI_VCOREIII_NUM_HW_CS;
    spi_vcoreiii_chipselect_set_gpio_cs(dev, on, gpio);
}

static void spi_vcoreiii_chipselect(struct spi_device *dev, int on)
{
    struct spi_vcoreiii *sp = spidev_to_sg(dev);
    struct spi_vcoreiii_controller_data *sd = dev->controller_data;
    void (*chipselect)(struct spi_device *dev, int on) = NULL;
    int use_gpio_cs = (dev->chip_select >= SPI_VCOREIII_NUM_HW_CS);

    // Std GPIO CS?
    if (use_gpio_cs) {
        chipselect = spi_vcoreiii_chipselect_gpio;
    }
    // Mux or custom CS?
    if (sd) {
        if (sd->chipselect) {
            chipselect = sd->chipselect;
        } else if (sd->mask_lo || sd->mask_hi) {
            chipselect = spi_vcoreiii_chipselect_mux;
        }
    }

    // This is more of a "hook" prior to changing CS
    if (sp->info->chipselect) {
        sp->info->chipselect(dev, on);
    }

    if(on) {
        sp->bb_cur =
                VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK_OE | /* SCK_OE */
                ((dev->mode & SPI_CPOL) ? VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_SCK : 0) | /* SCK */
                VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE ; /* SW Bitbang */
        /* Setup clock in right state before driving CS */
        writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        /* Activate CS */
        if (chipselect) {
            chipselect(dev, on);
        } else {
            // Assumes nCS for internal CS'es
            u32 mask =
                    VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS_OE(VTSS_BIT(dev->chip_select)) | /* CS_OE */
                    VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS(VTSS_BIT(dev->chip_select));
            sp->bb_cur |= mask;
            writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        }
        // Read back cached SW_MODE if "chipselect" hook has changed it
        sp->bb_cur = readl(VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        // CS setup/hold time
        if (sd && sd->cs_delay) {
            ndelay(sd->cs_delay);    // nsecs
        }
    } else {
        /* Drop CS */
        if (chipselect) {
            chipselect(dev, on);
        } else {
            // Clear bits to drop CS
            sp->bb_cur &= ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS;
            writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        }
        // CS setup/hold time
        if (sd && sd->cs_delay) {
            ndelay(sd->cs_delay);    // nsecs
        }
        /* Drop everything */
        sp->bb_cur = 0;
        writel(sp->bb_cur, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
    }
}

static int spi_vcoreiii_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spi_vcoreiii *sp;
	int err;

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_vcoreiii));
	if (!master) {
		dev_err(&pdev->dev, "failed to allocate spi master\n");
		return -ENOMEM;
	}

	sp = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, sp);
	sp->info = pdev->dev.platform_data;

	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.master->bus_num = 0;
	sp->bitbang.master->num_chipselect = SPI_VCOREIII_NUM_HW_CS + SPI_VCOREIII_NUM_GPIO_CS;
	sp->bitbang.chipselect = spi_vcoreiii_chipselect;

	sp->bitbang.txrx_word[SPI_MODE_0] = spi_vcoreiii_txrx_mode0;
	sp->bitbang.setup_transfer = spi_bitbang_setup_transfer;
	sp->bitbang.flags = SPI_CS_HIGH;

	err = spi_bitbang_start(&sp->bitbang);
	if (err)
		goto err_no_bitbang;

	return 0;

err_no_bitbang:
	spi_master_put(sp->bitbang.master);
	kfree(master);

	return err;
}

static int spi_vcoreiii_remove(struct platform_device *pdev)
{
	struct spi_vcoreiii *sp;
	struct spi_vcoreiii_platform_data *pdata;

	pdata = pdev->dev.platform_data;
	sp = platform_get_drvdata(pdev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

static void spi_vcoreiii_shutdown(struct platform_device *pdev)
{
        spi_vcoreiii_remove(pdev);
}

MODULE_ALIAS("platform:" SPI_VCOREIII_PLATDEV_NAME);
static struct platform_driver spi_vcoreiii_driver = {
	.driver		= {
		.name	= SPI_VCOREIII_PLATDEV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= spi_vcoreiii_probe,
	.remove		= spi_vcoreiii_remove,
	.shutdown 	= spi_vcoreiii_shutdown,
};
module_platform_driver(spi_vcoreiii_driver);

MODULE_AUTHOR("Lars Povlsen <lpovlsen at vitesse.com>");
MODULE_AUTHOR("Lars Povlsen");
MODULE_DESCRIPTION("VCore-III bitbanging SPI driver");
MODULE_LICENSE("GPL v2");
