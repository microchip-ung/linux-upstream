// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Microsemi SoCs serial gpio driver
 *
 * Author: <lars.povlsen@microchip.com>
 *
 * Copyright (c) 2018 Microsemi Corporation
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/clk.h>

#define MSCC_SGPIOS_PER_BANK	32
#define MSCC_SGPIO_BANK_DEPTH	4

enum {
	REG_INPUT_DATA,
	REG_PORT_CONFIG,
	REG_PORT_ENABLE,
	REG_SIO_CONFIG,
	REG_SIO_CLOCK,
	MAXREG
};

struct mscc_sgpio_bf {
	u8 beg;
	u8 end;
};

struct mscc_sgpio_props {
	const char *label;
	u8 regoff[MAXREG];
	struct mscc_sgpio_bf auto_repeat;
	struct mscc_sgpio_bf port_width;
	struct mscc_sgpio_bf clk_freq;
	struct mscc_sgpio_bf bit_source;
};

#define __M(bf)		GENMASK((bf).end, (bf).beg)
#define __F(bf, x)	(__M(bf) & ((x) << (bf).beg))
#define __X(bf, x)	(((x) >> (bf).beg) & GENMASK(((bf).end - (bf).beg), 0))

#define MSCC_M_CFG_SIO_AUTO_REPEAT(p)		BIT(p->props->auto_repeat.beg)
#define MSCC_F_CFG_SIO_PORT_WIDTH(p, x)		__F(p->props->port_width, x)
#define MSCC_M_CFG_SIO_PORT_WIDTH(p)		__M(p->props->port_width)
#define MSCC_F_CLOCK_SIO_CLK_FREQ(p, x)		__F(p->props->clk_freq, x)
#define MSCC_M_CLOCK_SIO_CLK_FREQ(p)		__M(p->props->clk_freq)
#define MSCC_F_PORT_CFG_BIT_SOURCE(p, x)	__F(p->props->bit_source, x)
#define MSCC_X_PORT_CFG_BIT_SOURCE(p, x)	__X(p->props->bit_source, x)

const struct mscc_sgpio_props props_luton = {
	.label = "luton-sgpio",
	.regoff = { 0x00, 0x09, 0x29, 0x2a, 0x2b },
	.auto_repeat = { 5, 5 },
	.port_width  = { 2, 3 },
	.clk_freq    = { 0, 11 },
	.bit_source  = { 0, 11 },
};

const struct mscc_sgpio_props props_ocelot = {
	.label = "ocelot-sgpio",
	.regoff = { 0x00, 0x06, 0x26, 0x04, 0x05 },
	.auto_repeat = { 10, 10 },
	.port_width  = {  7, 8  },
	.clk_freq    = {  8, 19 },
	.bit_source  = { 12, 23 },
};

struct mscc_sgpio_priv {
	struct gpio_chip gpio;
	u32 bitcount;
	u32 ports;
	u32 clock;
	u32 mode[MSCC_SGPIOS_PER_BANK];
	u32 __iomem *regs;
	const struct mscc_sgpio_props *props;
};

static inline u32 sgpio_readl(struct mscc_sgpio_priv *priv, u32 rno, u32 off)
{
	u32 __iomem *reg = &priv->regs[priv->props->regoff[rno] + off];

	return readl(reg);
}

static inline void sgpio_writel(struct mscc_sgpio_priv *priv,
				u32 val, u32 rno, u32 off)
{
	u32 __iomem *reg = &priv->regs[priv->props->regoff[rno] + off];

	writel(val, reg);
}

static void sgpio_clrsetbits(struct mscc_sgpio_priv *priv,
			     u32 rno, u32 off, u32 clear, u32 set)
{
	u32 __iomem *reg = &priv->regs[priv->props->regoff[rno] + off];
	u32 val = readl(reg);

	val &= ~clear;
	val |= set;

	writel(val, reg);
}

static int mscc_sgpio_direction_input(struct gpio_chip *gc, unsigned int gpio)
{
	struct mscc_sgpio_priv *priv = gpiochip_get_data(gc);

	u32 port = gpio % MSCC_SGPIOS_PER_BANK;
	u32 bit = gpio / MSCC_SGPIOS_PER_BANK;

	/* Set mode => input mode */
	priv->mode[port] |= BIT(bit);

	return 0;
}

static int mscc_sgpio_direction_output(struct gpio_chip *gc,
				       unsigned int gpio, int value)
{
	struct mscc_sgpio_priv *priv = gpiochip_get_data(gc);
	u32 port = gpio % MSCC_SGPIOS_PER_BANK;
	u32 bit = gpio / MSCC_SGPIOS_PER_BANK;
	u32 mask = 3 << (3 * bit);

	printk("set: port %d, bit %d, mask 0x%08x, value %d\n",
	       port, bit, mask, value);

	value = (value & 3) << (3 * bit);
	sgpio_clrsetbits(priv, REG_PORT_CONFIG, port,
			 MSCC_F_PORT_CFG_BIT_SOURCE(priv, mask),
			 MSCC_F_PORT_CFG_BIT_SOURCE(priv, value));

	/* Clear mode => output mode */
	priv->mode[port] &= ~BIT(bit);

	return 0;
}

static int mscc_sgpio_get_function(struct gpio_chip *gc, unsigned int gpio)
{
	struct mscc_sgpio_priv *priv = gpiochip_get_data(gc);
	u32 port = gpio % MSCC_SGPIOS_PER_BANK;
	u32 bit = gpio / MSCC_SGPIOS_PER_BANK;
	u32 val = priv->mode[port] & BIT(bit);

	return !!val;		/* 0=out, 1=in */
}

static void mscc_sgpio_set_value(struct gpio_chip *gc,
				unsigned int gpio, int value)
{
	mscc_sgpio_direction_output(gc, gpio, value);
}

static int mscc_sgpio_get_value(struct gpio_chip *gc, unsigned int gpio)
{
	struct mscc_sgpio_priv *priv = gpiochip_get_data(gc);
	u32 port = gpio % MSCC_SGPIOS_PER_BANK;
	u32 bit = gpio / MSCC_SGPIOS_PER_BANK;
	int ret;

	if (mscc_sgpio_get_function(gc, gpio)) {
		ret = !!(sgpio_readl(priv, REG_INPUT_DATA, bit) & BIT(port));
	} else {
		u32 portval = sgpio_readl(priv, REG_PORT_CONFIG, port);

		ret = MSCC_X_PORT_CFG_BIT_SOURCE(priv, portval);
		ret = !!(ret & (3 << (3 * bit)));
	}

	printk("get: gpio %d, port %d, bit %d, value %d\n",
	       gpio, port, bit, ret);

	return ret;
}

static int mscc_sgpio_get_count(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct of_phandle_args pinspec;
	int count, index, ret;

	for (count = 0, index = 0;; index++) {
		ret = of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3,
						       index, &pinspec);
		if (ret)
			break;

		dev_dbg(dev, "%s: Add %d gpios\n", __FUNCTION__, pinspec.args[2]);
		count += pinspec.args[2];
	}
	dev_dbg(dev, "%s: Have %d gpios\n", __FUNCTION__, count);
	return count;
}

static int mscc_sgpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mscc_sgpio_priv *priv;
	int div_clock = 0, ret, port;
	u32 val, ngpios;
	struct resource *regs;
	struct clk *clk;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "Failed to get clock\n");
		return PTR_ERR(clk);
	} else {
		div_clock = clk_get_rate(clk);
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);
	priv->props = of_device_get_match_data(dev);
	if (device_property_read_u32(dev, "mscc,sgpio-ports", &priv->ports))
		priv->ports = 0xFFFFFFFF;
	if (device_property_read_u32(dev, "mscc,sgpio-frequency", &priv->clock))
		priv->clock = 12500000;
	if (priv->clock <= 0 || priv->clock > div_clock) {
		dev_err(dev, "Invalid frequency %d\n", priv->clock);
		return -EINVAL;
	}
	if (device_property_read_u32(dev, "mscc,ngpios", &ngpios))
		ngpios = mscc_sgpio_get_count(dev);
	if (ngpios < 1 || ngpios > (4 * MSCC_SGPIOS_PER_BANK)) {
		dev_err(dev, "Invalid gpio count %d\n", ngpios);
		return -EINVAL;
	}

	priv->gpio.label		= priv->props->label;
	priv->gpio.parent		= dev;
	priv->gpio.of_node		= dev->of_node;
	priv->gpio.owner		= THIS_MODULE;
	priv->gpio.get_direction	= mscc_sgpio_get_function;
	priv->gpio.direction_input	= mscc_sgpio_direction_input;
	priv->gpio.direction_output	= mscc_sgpio_direction_output;
	priv->gpio.get			= mscc_sgpio_get_value,
	priv->gpio.set			= mscc_sgpio_set_value;
	priv->gpio.request		= gpiochip_generic_request;
	priv->gpio.free			= gpiochip_generic_free;
	priv->gpio.base			= -1;
	priv->gpio.ngpio		= ngpios;

	priv->bitcount = DIV_ROUND_UP(ngpios, MSCC_SGPIOS_PER_BANK);
	dev_dbg(dev, "probe: gpios = %d, bit-count = %d\n",
		ngpios, priv->bitcount);

	sgpio_clrsetbits(priv, REG_SIO_CONFIG, 0,
			 MSCC_M_CFG_SIO_PORT_WIDTH(priv),
			 MSCC_F_CFG_SIO_PORT_WIDTH(priv, priv->bitcount - 1) |
			 MSCC_M_CFG_SIO_AUTO_REPEAT(priv));
	val = div_clock / priv->clock;
	dev_dbg(dev, "probe: div-clock = %d KHz, freq = %d KHz, div = %d\n",
		div_clock / 1000, priv->clock / 1000, val);
	sgpio_clrsetbits(priv, REG_SIO_CLOCK, 0,
			 MSCC_M_CLOCK_SIO_CLK_FREQ(priv),
			 MSCC_F_CLOCK_SIO_CLK_FREQ(priv, val));

	for (port = 0; port < 32; port++)
		sgpio_writel(priv, 0, REG_PORT_CONFIG, port);
	sgpio_writel(priv, priv->ports, REG_PORT_ENABLE, 0);

	if ((ret = devm_gpiochip_add_data(dev, &priv->gpio, priv)) == 0) {
		dev_info(dev, "Registered %d GPIOs\n", ngpios);
	} else {
		dev_err(dev, "Failed to register: ret %d\n", ret);
	}
	return ret;
}

static const struct of_device_id mscc_sgpio_gpio_of_match[] = {
	{
		.compatible = "mscc,luton-sgpio",
		.data = &props_luton,
	}, {
		.compatible = "mscc,ocelot-sgpio",
		.data = &props_ocelot,
	}, {
		/* sentinel */
	}
};

static struct platform_driver mscc_sgpio_gpio_driver = {
	.driver = {
		.name = "mscc-sgpio",
		.of_match_table = mscc_sgpio_gpio_of_match,
	},
	.probe = mscc_sgpio_probe,
};
module_platform_driver(mscc_sgpio_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lars Povlsen <lars.povlsen@microchip.com>");
MODULE_DESCRIPTION("MSCC SoC serial GPIO driver");
