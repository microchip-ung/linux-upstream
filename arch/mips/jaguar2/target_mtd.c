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
#include <gpio.h>
#endif

#include <asm/mach-jaguar2/hardware.h>

static struct mtd_partition vcoreiii_partition_info[] = {
    [0] = {
        .name	= "rootfs_data",
        .offset	= 0,
        .size	= MTDPART_SIZ_FULL,
    },
};

static struct spi_vcoreiii_platform_data spi_jaguar2_cfg = {
    // .no_spi_delay = 1,
};

static struct platform_device jaguar2_spi = {
        .name             = SPI_VCOREIII_PLATDEV_NAME,
        .id               = 0,
        .dev = {
                .platform_data = &spi_jaguar2_cfg,
        },
};

static struct flash_platform_data jaguar2_spi_flash_data = {
	.type = "mx25l25635e",
	.name = "spi_flash",
        //        .read_mapped = 1,
        //        .phys_offset = 0x40000000,
        .use_4byte_commands = 1,
};

/* MMC-SPI driver */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
static int vcoreiii_mmc_spi_init(struct device *dev, 
                                 irqreturn_t (*detect_int)(int, void *), 
                                 void *data)
{
    /* Reserve SD/MMC CS pin */
    vcoreiii_gpio_set_alternate(18, 1); /* SI_nCS3/GPIO_18 */
    return 0;
}
#define VTSS_SPI_MMC_CD VCOREIII_SGPIO(0, 15, 1) // SGPIO:0 p15.1 = GPIO 111, 0 => NO Card detect
#define VTSS_SPI_MMC_WP VCOREIII_SGPIO(0, 15, 0) // SGPIO:0 p15.0 = GPIO 79, 0 => RO is OFF
static struct mmc_spi_platform_data jaguar2_mmc_spi_pdata = {
    .caps          = MMC_CAP_NEEDS_POLL,            /* No IRQ on SGPIO */
    .caps2         = MMC_CAP2_RO_ACTIVE_HIGH,
    .ocr_mask      = MMC_VDD_32_33 | MMC_VDD_33_34, /* default power */
    .detect_delay  = 100, /* msecs */
    .flags         = MMC_SPI_USE_CD_GPIO | MMC_SPI_USE_RO_GPIO,
    .cd_gpio	   = VTSS_SPI_MMC_CD,
    .ro_gpio       = VTSS_SPI_MMC_WP,
    .init          = vcoreiii_mmc_spi_init,
};
#endif

/* struct jaguar2_spi_chip_info { */
/*     void (*cs_control)(u32 command); */
/* }; */

/* static void jaguar2_mtd_cs_control(u32 command) */
/* { */
/*     u32 sw_mode = readl(VTSS_ICPU_CFG_SPI_MST_SW_MODE); */
/*     if (command) { */
/*         sw_mode |= */
/*                 VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS_OE(0) | /\* CS_OE *\/ */
/*                 VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS(0); */
/*         writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE); */
/*     } else { */
/*         /\* Drive CS low *\/ */
/*         sw_mode &= ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS; */
/*         writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE); */
/*         /\* Drop everything *\/ */
/*         sw_mode = 0; */
/*         writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE); */
/*     } */
/* } */

/* static struct jaguar2_spi_chip_info jaguar2_spi_flash_cs = { */
/*     .cs_control = jaguar2_mtd_cs_control, */
/* }; */


#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
static struct flash_platform_data jaguar2_spinand_flash_data = {
    .name = "spi_nand",
    .parts = vcoreiii_partition_info,
    .nr_parts = ARRAY_SIZE(vcoreiii_partition_info),
};
#endif


static struct spi_board_info jaguar2_spi_board_info[] __initdata = {
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &jaguar2_spi_flash_data,
                /* .controller_data = &jaguar2_spi_flash_cs,  /\* chip select control  *\/ */
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
        },
#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "mx35", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 1, /* Framework chip select. */
		.platform_data = &jaguar2_spinand_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
        },
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 3,
		.platform_data = &jaguar2_mmc_spi_pdata,
		.controller_data = NULL,
		.mode = SPI_MODE_0,
	},
#endif

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2)
        // sync
        {
                .modalias = "spidev",
                .max_speed_hz = 100000,
                .bus_num = 0,
                .chip_select = 1,
                .mode = SPI_MODE_0,
        },
        // cpld
        {
                .modalias = "spidev",
                .max_speed_hz = 100000,
                .bus_num = 0,
                .chip_select = SPI_VCOREIII_NUM_HW_CS + 18,    // GPIO 18 == CS
                .mode = SPI_MODE_0,
        },
#endif

};

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2)

#define NAND_ADDR_BIT_ALE (1 << 2)
#define NAND_ADDR_BIT_CLE (1 << 3)

/* hardware specific access to control-lines */
static void jaguar2_nand_cmd_ctl(struct mtd_info *mtd, int dat,
                                 unsigned int ctrl)
{
    struct nand_chip *this = mtd->priv;
    int offset = (int)this->priv;

    if (ctrl & NAND_CTRL_CHANGE) {
        offset = 0;
        if(ctrl & NAND_CLE) offset |= NAND_ADDR_BIT_CLE;
        if(ctrl & NAND_ALE) offset |= NAND_ADDR_BIT_ALE;
        this->priv = (void *)offset;
    }
    if (dat != NAND_CMD_NONE) {
        writeb(dat, this->IO_ADDR_W + offset);
    }
}

struct platform_nand_data jaguar2_nand_platdata = {
    .chip = {
        .nr_chips = 1,
        .chip_offset = 0,
        .nr_partitions = ARRAY_SIZE(vcoreiii_partition_info),
        .partitions = vcoreiii_partition_info,
        .chip_delay = 50,
    },
    .ctrl = {
        .cmd_ctrl = jaguar2_nand_cmd_ctl,
    },
};

#define JAGUAR2_NAND_CS	0 /* CS0 */

static struct resource jaguar2_nand_resource[] = {
    [0] = {
        .start = 0x50000000 + (JAGUAR2_NAND_CS*0x4000000), /* CS0 */
        .end   = 0x50000000 + (JAGUAR2_NAND_CS*0x4000000) + NAND_ADDR_BIT_CLE*2,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device jaguar2_nand = {
    .name		= "gen_nand",
    .num_resources	= ARRAY_SIZE(jaguar2_nand_resource),
    .resource	= jaguar2_nand_resource,
    .id		= -1,
    .dev		= {
        .platform_data = &jaguar2_nand_platdata,
    }
};
#endif

static int __init vcoreiii_mtd_init(void)
{
    /* Enable the PI interface */
    vcoreiii_io_set(VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL,  
                    VTSS_M_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL_IF_PI_MST_ENA);

    /* Slow down NAND CS via waitcc - appx 77ns */
    vcoreiii_io_mask_set(VTSS_ICPU_CFG_PI_MST_PI_MST_CTRL,
                         VTSS_M_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC,
                         VTSS_F_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC(8));

    platform_device_register(&jaguar2_spi);

#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
    vcoreiii_gpio_set_alternate(8, 1); /* SI_nCS1 */
#endif

    spi_register_board_info(jaguar2_spi_board_info, ARRAY_SIZE(jaguar2_spi_board_info));

    return 0;
}

static int __init vcoreiii_mtd_init_nand(void)
{
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2)
    platform_device_register(&jaguar2_nand);
#endif
    return 0;
}

module_init(vcoreiii_mtd_init)
late_initcall(vcoreiii_mtd_init_nand);
