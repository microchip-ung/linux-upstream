// SPDX-License-Identifier: GPL-2.0-only
/*
 * Memory-mapped interface driver for MSCC SoCs
 *
 * Copyright (c) 2020 Microsemi Corporation
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi-mem.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_mscc"

#define MAX_CS		4

#define MSCC_IF_SI_OWNER_MASK			GENMASK(1, 0)
#define MSCC_IF_SI_OWNER_SISL			0
#define MSCC_IF_SI_OWNER_SIBM			1
#define MSCC_IF_SI_OWNER_SIMC			2

#define MSCC_SPI_MST_SW_MODE			0x14

struct dw_spi_mscc_props {
	const char *syscon_name;
	u32 general_ctrl_off;
	u32 si_owner_bit, si_owner2_bit;
	u32 pinctrl_bit_off;
	u32 cs_bit_off;
	u32 ss_force_ena_off;
	u32 ss_force_val_off;
	u32 bootmaster_cs;
};

struct dw_spi_mscc {
	struct dw_spi  dws;
	struct clk     *clk;
	void __iomem   *read_map;
	struct regmap			*syscon;
	void __iomem			*spi_mst;
	const struct dw_spi_mscc_props	*props;
	u32				gen_owner;
	u32				if2mask;
};

static const struct dw_spi_mscc_props dw_spi_mscc_props_ocelot = {
	.syscon_name		= "mscc,ocelot-cpu-syscon",
	.general_ctrl_off	= 0x24,
	.si_owner_bit		= 4,
	.pinctrl_bit_off	= 13,
	.cs_bit_off		= 5,
	.bootmaster_cs		= 0,
};

static const struct dw_spi_mscc_props dw_spi_mscc_props_jaguar2 = {
	.syscon_name		= "mscc,ocelot-cpu-syscon",
	.general_ctrl_off	= 0x24,
	.si_owner_bit		= 6,
	.pinctrl_bit_off	= 13,
	.cs_bit_off		= 5,
	.bootmaster_cs		= 0,
};

static const struct dw_spi_mscc_props dw_spi_mscc_props_fireant = {
	.syscon_name		= "mscc,fireant-cpu-syscon",
	.general_ctrl_off	= 0x88,
	.si_owner_bit		= 6,
	.si_owner2_bit		= 4,
	.ss_force_ena_off	= 0xa4,
	.ss_force_val_off	= 0xa8,
	.bootmaster_cs		= 0,
};

/*
 * Set the owner of the SPI interface
 */
static void dw_spi_mscc_set_owner(struct dw_spi_mscc *dwsmscc,
				  const struct dw_spi_mscc_props *props,
				  u8 owner, u8 owner2)
{
	u32 val, msk;

	val = (owner << props->si_owner_bit);
	msk = (MSCC_IF_SI_OWNER_MASK << props->si_owner_bit);
	if (props->si_owner2_bit) {
		val |= owner2 << props->si_owner2_bit;
		msk |= (MSCC_IF_SI_OWNER_MASK << props->si_owner2_bit);
	}
	if (dwsmscc->gen_owner != val) {
		regmap_update_bits(dwsmscc->syscon, props->general_ctrl_off,
				   msk, val);
		dwsmscc->gen_owner = val;
	}
}

static void dw_spi_mscc_set_cs_owner(struct dw_spi_mscc *dwsmscc,
				     const struct dw_spi_mscc_props *props,
				     u8 cs, u8 owner)
{
	u8 dummy = (owner == MSCC_IF_SI_OWNER_SIBM ?
		    MSCC_IF_SI_OWNER_SIMC : MSCC_IF_SI_OWNER_SIBM);
	if (props->si_owner2_bit && (dwsmscc->if2mask & BIT(cs))) {
		/* SPI2 */
		dw_spi_mscc_set_owner(dwsmscc, props, dummy, owner);
	} else {
		/* SPI1 */
		dw_spi_mscc_set_owner(dwsmscc, props, owner, dummy);
	}
}

/*
 * The Designware SPI controller (referred to as master in the
 * documentation) automatically deasserts chip select when the tx fifo
 * is empty. The chip selects then needs to be either driven as GPIOs
 * or, for the first 4 using the the SPI boot controller
 * registers. the final chip select is an OR gate between the
 * Designware SPI controller and the SPI boot controller.  nselect is
 * an active low signal
 */
static void dw_spi_mscc_set_cs(struct spi_device *spi, bool enable)
{
	struct dw_spi *dws = spi_master_get_devdata(spi->master);
	struct dw_spi_mscc *dwsmscc = container_of(dws, struct dw_spi_mscc,
						   dws);
	const struct dw_spi_mscc_props *props = dwsmscc->props;
	u8 cs = spi->chip_select;

	if (enable)
		dw_spi_mscc_set_cs_owner(dwsmscc, props, cs,
					 MSCC_IF_SI_OWNER_SIMC);

	if (dwsmscc->spi_mst && (cs < MAX_CS)) {
		u32 sw_mode;

		if (enable)
			sw_mode = BIT(props->pinctrl_bit_off) |
				(BIT(cs) << props->cs_bit_off);
		else
			sw_mode = 0;
		writel(sw_mode, dwsmscc->spi_mst + MSCC_SPI_MST_SW_MODE);
	} else if (props->ss_force_ena_off) {
		if (enable) {
			/* Ensure CS toggles, so start off all disabled */
			regmap_write(dwsmscc->syscon, props->ss_force_val_off,
				     ~0);
			/* CS override drive enable */
			regmap_write(dwsmscc->syscon, props->ss_force_ena_off,
				     1);
			/* Allow settle */
			udelay(1);
			/* Now set CSx enabled */
			regmap_write(dwsmscc->syscon, props->ss_force_val_off,
				     ~BIT(cs));
		} else {
			/* CS value */
			regmap_write(dwsmscc->syscon, props->ss_force_val_off,
				     ~0);
			/* CS override drive disable */
			regmap_write(dwsmscc->syscon, props->ss_force_ena_off,
				     0);
		}
	}

	dw_spi_set_cs(spi, enable);
}

static int vcoreiii_bootmaster_exec_mem_op(struct spi_mem *mem,
					   const struct spi_mem_op *op)
{
	struct spi_device *spi = mem->spi;
	int ret = -ENOTSUPP;

	/* Only reads, addrsize 1..4 */
	if (!op->data.nbytes || !op->addr.nbytes || op->addr.nbytes > 4 ||
	    op->data.dir != SPI_MEM_DATA_IN)
		return ret;

	/* Only handle (normal+fast) 3/4 bytes read */
	if (op->cmd.opcode != SPINOR_OP_READ &&
	    op->cmd.opcode != SPINOR_OP_READ_FAST &&
	    op->cmd.opcode != SPINOR_OP_READ_4B &&
	    op->cmd.opcode != SPINOR_OP_READ_FAST_4B)
		return ret;

	/* CS0..3, only 16M reach */
	if ((spi->chip_select < MAX_CS) &&
	    (op->addr.val + op->data.nbytes) < SZ_16M) {
		struct dw_spi *dws = spi_master_get_devdata(spi->master);
		struct dw_spi_mscc *dwsmscc = container_of(dws,
							   struct dw_spi_mscc,
							   dws);
		const struct dw_spi_mscc_props *props = dwsmscc->props;
		u8 __iomem *src = dwsmscc->read_map +
			(spi->chip_select * SZ_16M) + op->addr.val;

		if (props->bootmaster_cs != spi->chip_select)
			return ret;

		/* Make boot master owner of SI interface */
		dw_spi_mscc_set_cs_owner(dwsmscc, props, spi->chip_select,
					 MSCC_IF_SI_OWNER_SIBM);
		memcpy(op->data.buf.in, src, op->data.nbytes);
		ret = op->data.nbytes;
	}
	return ret;
}

static const struct spi_controller_mem_ops vcoreiii_bootmaster_mem_ops = {
	.exec_op = vcoreiii_bootmaster_exec_mem_op,
};

static int dw_spi_mscc_init(struct platform_device *pdev,
			    struct dw_spi *dws,
			    struct dw_spi_mscc *dwsmscc,
			    const struct dw_spi_mscc_props *props)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res && resource_size(res) > 0) {
		dwsmscc->spi_mst = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(dwsmscc->spi_mst)) {
			dev_err(&pdev->dev, "SPI_MST region map failed\n");
			return PTR_ERR(dwsmscc->spi_mst);
		}
	}

	/* See if we have a direct read window */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res && resource_size(res) >= (SZ_16M*MAX_CS)) {
		void __iomem *ptr = devm_ioremap_resource(&pdev->dev, res);

		if (!IS_ERR(ptr)) {
			dwsmscc->read_map = ptr;
			dws->mem_ops = &vcoreiii_bootmaster_mem_ops;
			dev_info(&pdev->dev, "Enabling fast memory operations\n");
		}
	}

	dwsmscc->syscon =
		syscon_regmap_lookup_by_compatible(props->syscon_name);
	if (IS_ERR(dwsmscc->syscon)) {
		dev_err(&pdev->dev, "No syscon map %s\n", props->syscon_name);
		return PTR_ERR(dwsmscc->syscon);
	}
	dwsmscc->props = props;

	/* Deassert all CS */
	if (dwsmscc->spi_mst)
		writel(0, dwsmscc->spi_mst + MSCC_SPI_MST_SW_MODE);

	/* SPI2 mapping bitmask */
	device_property_read_u32(&pdev->dev, "interface-mapping-mask",
				 &dwsmscc->if2mask);

	dwsmscc->dws.set_cs = dw_spi_mscc_set_cs;

	return 0;
}

static int dw_spi_mscc_probe(struct platform_device *pdev)
{
	const struct dw_spi_mscc_props *props;
	struct dw_spi_mscc *dwsmscc;
	struct dw_spi *dws;
	int ret;
	int num_cs;

	dwsmscc = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mscc),
			GFP_KERNEL);
	if (!dwsmscc)
		return -ENOMEM;

	dws = &dwsmscc->dws;

	/* Get basic io resource and map it */
	dws->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0)
		dev_info(&pdev->dev, "no irq, using polled mode\n");

	dwsmscc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmscc->clk))
		return PTR_ERR(dwsmscc->clk);
	ret = clk_prepare_enable(dwsmscc->clk);
	if (ret)
		return ret;

	dws->bus_num = pdev->id;

	dws->max_freq = clk_get_rate(dwsmscc->clk);

	device_property_read_u32(&pdev->dev, "reg-io-width",
				 &dws->reg_io_width);

	num_cs = MAX_CS;

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
	if (props)
		ret = dw_spi_mscc_init(pdev, dws, dwsmscc, props);
	else
		ret = -EINVAL;
	if (ret)
		goto out;

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, dwsmscc);
	return 0;

out:
	clk_disable_unprepare(dwsmscc->clk);
	return ret;
}

static int dw_spi_mscc_remove(struct platform_device *pdev)
{
	struct dw_spi_mscc *dwsmscc = platform_get_drvdata(pdev);

	dw_spi_remove_host(&dwsmscc->dws);
	clk_disable_unprepare(dwsmscc->clk);

	return 0;
}

static const struct of_device_id dw_spi_mscc_of_match[] = {
	{ .compatible = "mscc,ocelot-spi", .data = &dw_spi_mscc_props_ocelot},
	{ .compatible = "mscc,jaguar2-spi", .data = &dw_spi_mscc_props_jaguar2},
	{ .compatible = "mscc,fireant-spi", .data = &dw_spi_mscc_props_fireant},
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_mscc_of_match);

static struct platform_driver dw_spi_mscc_driver = {
	.probe		= dw_spi_mscc_probe,
	.remove		= dw_spi_mscc_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_mscc_of_match,
	},
};
module_platform_driver(dw_spi_mscc_driver);

MODULE_AUTHOR("Lars Povlsen <lars.povlsen@microchip.com>");
MODULE_DESCRIPTION("Memory-mapped I/O interface DW SPI driver for MSCC SoCs");
MODULE_LICENSE("GPL v2");
