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
#include <linux/spi/flash.h>

#include <asm/mach-vcoreiii/hardware.h>
#include <asm/vcoreiii-gpio.h>
#include <linux/sizes.h>

static struct platform_device vcoreiii_spi = {
        .name             = SPI_VCOREIII_PLATDEV_NAME,
        .id               = 0,
};

static struct flash_platform_data vcoreiii_spi_flash_data = {
	.type = "jedec,spi-nor",
	.name = "spi_flash",
        .read_mapped = 1,
        .phys_offset = 0x40000000,
        .phys_length = SZ_16M,
        .use_4byte_commands = 1,
};

static struct spi_board_info vcoreiii_spi_board_info[] __initdata = {
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 50000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &vcoreiii_spi_flash_data,
		.mode = SPI_MODE_0, /* CPOL=0, CPHA=0 */
	},
};

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
#define NAND_ADDR_BIT_ALE (1 << 13)
#define NAND_ADDR_BIT_CLE (1 << 14)
#else
#define NAND_ADDR_BIT_ALE (1 << 2)
#define NAND_ADDR_BIT_CLE (1 << 3)
#endif

/* hardware specific access to control-lines */
static void vcoreiii_nand_cmd_ctl(struct mtd_info *mtd, int cmd,
				  unsigned int ctrl)
{
	struct nand_chip *this = mtd_to_nand(mtd);
	if (ctrl & NAND_CTRL_CHANGE) {
            u32 ioaddr = (u32) this->IO_ADDR_R;
            if(ctrl & NAND_CLE) {
                ioaddr |= NAND_ADDR_BIT_CLE;
            } else if(ctrl & NAND_ALE) {
                ioaddr |= NAND_ADDR_BIT_ALE;
            }
            this->IO_ADDR_W = (void __iomem *)ioaddr;
	}
	if (cmd != NAND_CMD_NONE) {
            __raw_writeb(cmd, this->IO_ADDR_W);
            wmb();
	}
}

static struct mtd_partition vcoreiii_partition_info[] = {
	[0] = {
		.name	= "rootfs_data",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

struct platform_nand_data vcoreiii_nand_platdata = {
	.chip = {
		.nr_chips = 1,
		.chip_offset = 0,
		.nr_partitions = ARRAY_SIZE(vcoreiii_partition_info),
		.partitions = vcoreiii_partition_info,
                .chip_delay = 100,    // MX30LF1GE8AB with internal ECC has 45-70 us tR_ECC
	},
	.ctrl = {
		.cmd_ctrl = vcoreiii_nand_cmd_ctl,
	},
};

#ifdef CONFIG_VTSS_VCOREIII_JAGUAR
#define VCOREIII_NAND_CS	1 /* CS1 */
#else
#define VCOREIII_NAND_CS	0 /* CS0 */
#endif

static struct resource vcoreiii_nand_resource[] = {
    [0] = {
        .start = 0x50000000 + (VCOREIII_NAND_CS*0x4000000), /* CS1 */
        .end   = 0x50000000 + (VCOREIII_NAND_CS*0x4000000) + NAND_ADDR_BIT_CLE*2,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vcoreiii_nand = {
	.name		= "gen_nand",
	.num_resources	= ARRAY_SIZE(vcoreiii_nand_resource),
	.resource	= vcoreiii_nand_resource,
	.id		= -1,
	.dev		= {
		.platform_data = &vcoreiii_nand_platdata,
	}
};

static int __init vcoreiii_mtd_init(void)
{

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
	vcoreiii_gpio_set_alternate(19, 1);
#endif

        /* Slow down NAND CS via waitcc - appx 77ns */
	vcoreiii_io_mask_set(VTSS_ICPU_CFG_PI_MST_PI_MST_CTRL(VCOREIII_NAND_CS),
                             VTSS_M_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC,
                             VTSS_F_ICPU_CFG_PI_MST_PI_MST_CTRL_WAITCC(8));

	platform_device_register(&vcoreiii_spi);

	spi_register_board_info(vcoreiii_spi_board_info, ARRAY_SIZE(vcoreiii_spi_board_info));

	return 0;
}

static int __init vcoreiii_mtd_init_nand(void)
{
	if (!IS_ERR(get_mtd_device_nm("rootfs_data"))) {
		pr_warn("mtd('rootfs_data') seen, disabling NAND\n");
	} else {
		platform_device_register(&vcoreiii_nand);
	}
	return 0;
}

module_init(vcoreiii_mtd_init);
late_initcall(vcoreiii_mtd_init_nand);
