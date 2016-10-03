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
 *
 * VCore-III MTD platform driver
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mtd/mtd-abi.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_vcoreiii.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/flash.h>
#include <linux/semaphore.h>
#include <linux/sizes.h>

#include <asm/mach-jaguar2/hardware.h>
#include <asm/vcoreiii-gpio.h>

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2_ARCH) && !defined(CONFIG_MTD_NAND_SPINAND_MX35)
#define JR2_PI_NAND 1
#endif

static struct mtd_partition vcoreiii_partition_info[] = {
    [0] = {
        .name	= "rootfs_data",
        .offset	= 0,
        .size	= MTDPART_SIZ_FULL,
    },
};

#if defined(CONFIG_VTSS_VCOREIII_SERVALT)

DEFINE_SEMAPHORE(platform_gpio8_lock);

static void platform_chipselect(struct spi_device *spi, int on)
{
    if (spi->chip_select == 1) {    // NAND == SI_nCS1 == GPIO8
        if (on == BITBANG_CS_ACTIVE) {
            down(&platform_gpio8_lock);
            vcoreiii_gpio_set_mode(8, VCOREIII_GPIO_MODE_ALT1);    // ALT 0b'01'
        } else {
            vcoreiii_gpio_set_mode(8, VCOREIII_GPIO_MODE_ALT3);    // ALT 0b'00'
            up(&platform_gpio8_lock);
        }
    }
}
#endif

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
        .read_mapped = 1,
        .phys_offset = 0x40000000,
        .use_4byte_commands = 1,
};

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


#if defined(CONFIG_MTD_NAND_SPINAND_MX35)

static struct flash_platform_data jaguar2_spinand_flash_data = {
    .name = "spi_nand",
    .parts = vcoreiii_partition_info,
    .nr_parts = ARRAY_SIZE(vcoreiii_partition_info),
};
#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
#define SPINAND_CS   1
#define SPINAND_GPIO 8
#else    // Serval2
#define SPINAND_CS   3
#define SPINAND_GPIO 18
#endif
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
#if defined(CONFIG_MTD_NAND_SPINAND_MX35)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "mx35", /* Name of spi_driver for this device */
		.max_speed_hz = 104000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = SPINAND_CS, /* Framework chip select. */
		.platform_data = &jaguar2_spinand_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
        },
#endif

};

#if defined(JR2_PI_NAND)

#define NAND_ADDR_BIT_ALE (1 << 2)
#define NAND_ADDR_BIT_CLE (1 << 3)

/* hardware specific access to control-lines */
static void jaguar2_nand_cmd_ctl(struct mtd_info *mtd, int cmd,
                                 unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	if (ctrl & NAND_CTRL_CHANGE) {
            u32 ioaddr =  (u32) this->IO_ADDR_W;
            ioaddr &= ~(NAND_ADDR_BIT_CLE|NAND_ADDR_BIT_ALE);
            if(ctrl & NAND_CLE) ioaddr |= NAND_ADDR_BIT_CLE;
            if(ctrl & NAND_ALE) ioaddr |= NAND_ADDR_BIT_ALE;
            this->IO_ADDR_R = this->IO_ADDR_W = (void __iomem *)ioaddr;
	}
	if (cmd != NAND_CMD_NONE) {
            __raw_writeb(cmd, this->IO_ADDR_W);
            wmb();
	}
}

struct platform_nand_data jaguar2_nand_platdata = {
    .chip = {
        .nr_chips = 1,
        .chip_offset = 0,
        .nr_partitions = ARRAY_SIZE(vcoreiii_partition_info),
        .partitions = vcoreiii_partition_info,
        .chip_delay = 100,    // MX30LF1GE8AB with internal ECC has 45-70 us tR_ECC
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
#if defined(JR2_PI_NAND)
    /* Enable the PI interface */
    vcoreiii_io_set(VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL,
                    VTSS_M_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL_IF_PI_MST_ENA);

    /* Slow down NAND CS via waitcc - appx 77ns */
    vcoreiii_io_mask_set(VTSS_ICPU_CFG_PI_MST_PI_MST_CTRL,
                         VTSS_M_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC,
                         VTSS_F_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC(8));
#endif

#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
    if (vcoreiii_chid_rev == 0) {
        // Chip issue wrt I2C MUX
        printk(KERN_INFO "SPI: VSC%04x rev %c - CS1 workaround applied\n", vcoreiii_chip_id, vcoreiii_chid_rev + 'A');
        spi_jaguar2_cfg.chipselect = platform_chipselect;
    }
#endif

    platform_device_register(&jaguar2_spi);

#if defined(SPINAND_GPIO)
    vcoreiii_gpio_set_alternate(SPINAND_GPIO, 1); /* SI_nCSX */
#endif

    spi_register_board_info(jaguar2_spi_board_info, ARRAY_SIZE(jaguar2_spi_board_info));

    return 0;
}
module_init(vcoreiii_mtd_init);

#if defined(JR2_PI_NAND)
static int __init vcoreiii_mtd_init_nand(void)
{
    platform_device_register(&jaguar2_nand);
    return 0;
}

late_initcall(vcoreiii_mtd_init_nand);
#endif
