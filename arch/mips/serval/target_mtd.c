/*
 *
 * VCore-III MTD platform driver
 *
 * Copyright (C) 2010 Vitesse Semiconductor Inc.
 * Author: Lars Povlsen (lpovlsen@vitesse.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * VITESSE SEMICONDUCTOR INC SHALL HAVE NO LIABILITY WHATSOEVER OF ANY
 * KIND ARISING OUT OF OR RELATED TO THE PROGRAM OR THE OPEN SOURCE
 * MATERIALS UNDER ANY THEORY OF LIABILITY.
 *
 */
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mtd/mtd-abi.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_vcoreiii.h>
#include <linux/spi/flash.h>

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
#include <linux/mmc/host.h>
#include <linux/spi/mmc_spi.h>
#endif

#include <asm/mach-serval/hardware.h>

static struct spi_vcoreiii_platform_data spi_serval_cfg = {
	// .no_spi_delay = 1,
};

static struct platform_device serval_spi = {
        .name             = SPI_VCOREIII_PLATDEV_NAME,
        .id               = 0,
        .dev = {
                .platform_data = &spi_serval_cfg,
        },
};

static struct flash_platform_data serval_spi_flash_data = {
	.type = "mx25l12805d",
	.name = "spi_flash",
        .read_mapped = 1,
        .phys_offset = 0x40000000,
};

/* MMC-SPI driver */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)

#define VTSS_SPI_MMC_CD (32+32+10)    // SGPIO p10.1 = GPIO 74, 0 => NO Card detect
#define VTSS_SPI_MMC_WP (32+ 0+10)    // SGPIO p10.0 = GPIO 42, 0 => RO is OFF
static struct mmc_spi_platform_data serval_mmc_spi_pdata = {
    .caps          = MMC_CAP_NEEDS_POLL,            /* No IRQ on SGPIO */
    .caps2         = MMC_CAP2_RO_ACTIVE_HIGH,
    .ocr_mask      = MMC_VDD_32_33 | MMC_VDD_33_34, /* default power */
    .detect_delay  = 100, /* msecs */
    .powerup_msecs = 100, /* msecs */
    .flags         = MMC_SPI_USE_CD_GPIO | MMC_SPI_USE_RO_GPIO,
    .cd_gpio	   = VTSS_SPI_MMC_CD,
    .ro_gpio       = VTSS_SPI_MMC_WP,
};
#endif

#if defined(CONFIG_MTD_NAND_PLATFORM)
static struct mtd_partition vcoreiii_partition_info[] = {
    [0] = {
        .name	= "rootfs_data",
        .offset	= 0,
        .size	= MTDPART_SIZ_FULL,
    },
};

static struct flash_platform_data serval_spinand_flash_data = {
    .name = "spi_nand",
    .parts = vcoreiii_partition_info,
    .nr_parts = ARRAY_SIZE(vcoreiii_partition_info),
};
#endif

static struct spi_board_info serval_spi_board_info[] __initdata = {
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &serval_spi_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
        },
#if defined(CONFIG_MTD_NAND_PLATFORM)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "mx35", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 1, /* Framework chip select. */
		.platform_data = &serval_spinand_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
        },
#endif
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 1,
		.platform_data = &serval_mmc_spi_pdata,
		.controller_data = NULL,
		.mode = SPI_MODE_0,
	},
#endif

        // sync
        {
                .modalias = "spidev",
                .max_speed_hz = 100000,
                .bus_num = 0,
                .chip_select = SPI_VCOREIII_NUM_HW_CS + 6,    // GPIO 6 == CS
                .mode = SPI_MODE_0,
        },
        // t1e1j1
        {
                .modalias = "spidev",
                .max_speed_hz = 100000,
                .bus_num = 0,
                .chip_select = SPI_VCOREIII_NUM_HW_CS + 17,    // GPIO 17 == CS
                .mode = SPI_MODE_0,
        },

};

static int __init vcoreiii_mtd_init(void)
{
	platform_device_register(&serval_spi);

#if defined(CONFIG_MTD_NAND_PLATFORM) || defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
        vcoreiii_gpio_set_alternate(8, 1); /* SI_nEN1/GPIO_8 */
#endif

	spi_register_board_info(serval_spi_board_info, ARRAY_SIZE(serval_spi_board_info));

	return 0;
}

module_init(vcoreiii_mtd_init)
