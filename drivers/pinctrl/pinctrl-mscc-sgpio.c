// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Microsemi SoCs serial gpio driver
 *
 * Author: <lars.povlsen@microchip.com>
 *
 * Copyright (c) 2018 Microsemi Corporation
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>

#include <dt-bindings/gpio/mscc-sgpio.h>

#include "core.h"
#include "pinconf.h"
#include "pinmux.h"

#define PIN_NAME_LEN	(sizeof("SGPIO_pXXbY")+1)

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

static const char * const functions[] = { "gpio" };

struct mscc_sgpio_priv {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct gpio_chip gpio;
	u32 bitcount;
	u32 ports;
	u32 clock;
	u32 mode[MSCC_SGPIOS_PER_BANK];
	u32 __iomem *regs;
	struct pinctrl_desc *desc;
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

static void sgpio_output_set(struct mscc_sgpio_priv *priv,
			     int pin, int value)
{
	u32 port = pin % MSCC_SGPIOS_PER_BANK;
	u32 bit = pin / MSCC_SGPIOS_PER_BANK;
	u32 mask = 3 << (3 * bit);

	dev_dbg(priv->dev, "%s: port %d, bit %d, value %d\n",
		__func__, port, bit, value);

	value = (value & 3) << (3 * bit);
	sgpio_clrsetbits(priv, REG_PORT_CONFIG, port,
			 MSCC_F_PORT_CFG_BIT_SOURCE(priv, mask),
			 MSCC_F_PORT_CFG_BIT_SOURCE(priv, value));
}

static int sgpio_output_get(struct mscc_sgpio_priv *priv, int pin)
{
	u32 port = pin % MSCC_SGPIOS_PER_BANK;
	u32 bit = pin / MSCC_SGPIOS_PER_BANK;
	u32 portval = sgpio_readl(priv, REG_PORT_CONFIG, port);
	int ret;

	ret = MSCC_X_PORT_CFG_BIT_SOURCE(priv, portval);
	ret = !!(ret & (3 << (3 * bit)));

	dev_dbg(priv->dev, "%s: port %d, bit %d, value %d\n",
		__func__, port, bit, ret);

	return ret;
}

static int sgpio_input_get(struct mscc_sgpio_priv *priv, int pin)
{
	u32 port = pin % MSCC_SGPIOS_PER_BANK;
	u32 bit = pin / MSCC_SGPIOS_PER_BANK;
	int ret;

	ret = !!(sgpio_readl(priv, REG_INPUT_DATA, bit) & BIT(port));

	dev_dbg(priv->dev, "%s: port %d, bit %d, value %d\n",
		__func__, port, bit, ret);

	return ret;
}

static int sgpio_pinconf_get(struct pinctrl_dev *pctldev,
			     unsigned int pin, unsigned long *config)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	u32 param = pinconf_to_config_param(*config);
	int val;

	switch (param) {
	case PIN_CONFIG_INPUT_ENABLE:
		val = false;
		break;

	case PIN_CONFIG_OUTPUT_ENABLE:
		val = true;
		break;

	case PIN_CONFIG_OUTPUT:
		val = sgpio_output_get(priv, pin);
		break;

	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, val);

	return 0;
}

noinline int sgpio_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *configs, unsigned int num_configs)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	u32 param, arg;
	int cfg, err = 0;

	for (cfg = 0; cfg < num_configs; cfg++) {
		param = pinconf_to_config_param(configs[cfg]);
		arg = pinconf_to_config_argument(configs[cfg]);

		switch (param) {
		case PIN_CONFIG_OUTPUT:
			sgpio_output_set(priv, pin, arg);
			break;

		default:
			err = -ENOTSUPP;
		}
	}

	return err;
}

static const struct pinconf_ops sgpio_confops = {
	.is_generic = true,
	.pin_config_get = sgpio_pinconf_get,
	.pin_config_set = sgpio_pinconf_set,
	.pin_config_config_dbg_show = pinconf_generic_dump_config,
};

static int sgpio_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(priv->dev, "%s\n", __func__);
	return 1;
}

static const char *sgpio_get_function_name(struct pinctrl_dev *pctldev,
					   unsigned int function)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(priv->dev, "%s\n", __func__);

	return functions[0];
}

static int sgpio_get_function_groups(struct pinctrl_dev *pctldev,
				      unsigned int function,
				      const char *const **groups,
				      unsigned *const num_groups)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(priv->dev, "%s\n", __func__);

	*groups  = functions;
	*num_groups = ARRAY_SIZE(functions);

	return 0;
}

static int sgpio_pinmux_set_mux(struct pinctrl_dev *pctldev,
				unsigned int selector, unsigned int group)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(priv->dev, "%s: sel %d grp %d\n", __func__, selector, group);

	return 0;
}

static int sgpio_gpio_set_direction(struct pinctrl_dev *pctldev,
				    struct pinctrl_gpio_range *range,
				    unsigned int pin, bool input)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(priv->dev, "%s: pin %d input %d\n", __func__, pin, input);

	return 0;
}

static int sgpio_gpio_request_enable(struct pinctrl_dev *pctldev,
				     struct pinctrl_gpio_range *range,
				     unsigned int offset)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	if ((priv->ports & BIT(offset)) == 0) {
		dev_warn(priv->dev, "%s: Request port pin %d is not activated\n",
			 __func__, offset);
	}

	return 0;
}

static const struct pinmux_ops sgpio_pmx_ops = {
	.get_functions_count = sgpio_get_functions_count,
	.get_function_name = sgpio_get_function_name,
	.get_function_groups = sgpio_get_function_groups,
	.set_mux = sgpio_pinmux_set_mux,
	.gpio_set_direction = sgpio_gpio_set_direction,
	.gpio_request_enable = sgpio_gpio_request_enable,
};

static int sgpio_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->desc->npins;
}

static const char *sgpio_pctl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned int group)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->desc->pins[group].name;
}

static int sgpio_pctl_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned int group,
				      const unsigned int **pins,
				      unsigned int *num_pins)
{
	struct mscc_sgpio_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	*pins = &priv->desc->pins[group].number;
	*num_pins = 1;

	return 0;
}

static const struct pinctrl_ops sgpio_pctl_ops = {
	.get_groups_count = sgpio_pctl_get_groups_count,
	.get_group_name = sgpio_pctl_get_group_name,
	.get_group_pins = sgpio_pctl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static struct pinctrl_desc sgpio_desc = {
	.name = "sgpio-pinctrl",
	.pctlops = &sgpio_pctl_ops,
	.pmxops = &sgpio_pmx_ops,
	.confops = &sgpio_confops,
	.owner = THIS_MODULE,
};

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

	sgpio_output_set(priv, gpio, value);

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
	const struct pinctrl_pin_desc *pin = &priv->desc->pins[gpio];
	int ret;

	if (mscc_sgpio_get_function(gc, gpio))
		ret = sgpio_input_get(priv, gpio);
	else
		ret = sgpio_output_get(priv, gpio);

	dev_dbg(priv->dev, "get: gpio %d (%s), value %d\n",
		gpio, pin->name, ret);

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

		dev_dbg(dev, "%s: Add %d gpios\n", __func__, pinspec.args[2]);
		count += pinspec.args[2];
	}
	dev_dbg(dev, "%s: Have %d gpios\n", __func__, count);
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
	struct pinctrl_pin_desc *pins;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "Failed to get clock\n");
		return PTR_ERR(clk);
	}
	div_clock = clk_get_rate(clk);

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
	ngpios = mscc_sgpio_get_count(dev);
	if (ngpios < 1 ||
	    ngpios > (MSCC_SGPIO_BANK_DEPTH * MSCC_SGPIOS_PER_BANK)) {
		dev_err(dev, "Invalid gpio count %d\n", ngpios);
		return -EINVAL;
	}

	pins = devm_kzalloc(dev, sizeof(*pins)*ngpios, GFP_KERNEL);
	if (pins) {
		int i;
		char *p, *names;

		names = devm_kzalloc(dev, PIN_NAME_LEN*ngpios, GFP_KERNEL);

		if (!names)
			return -ENOMEM;

		sgpio_desc.npins = ngpios;
		sgpio_desc.pins = pins;

		for (p = names, i = 0; i < ngpios; i++, p += PIN_NAME_LEN) {
			snprintf(p, PIN_NAME_LEN, "SGPIO_p%db%d",
				 i % MSCC_SGPIOS_PER_BANK,
				 i / MSCC_SGPIOS_PER_BANK);
			pins[i].number = i;
			pins[i].name = p;
		}
	} else
		return -ENOMEM;

	priv->desc = &sgpio_desc;
	priv->pctl = devm_pinctrl_register(&pdev->dev, priv->desc, priv);
	if (IS_ERR(priv->pctl)) {
		dev_err(&pdev->dev, "Failed to register pinctrl\n");
		return PTR_ERR(priv->pctl);
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

	for (port = 0; port < MSCC_SGPIOS_PER_BANK; port++)
		sgpio_writel(priv, 0, REG_PORT_CONFIG, port);
	sgpio_writel(priv, priv->ports, REG_PORT_ENABLE, 0);

	ret = devm_gpiochip_add_data(dev, &priv->gpio, priv);
	if (ret == 0)
		dev_info(dev, "Registered %d GPIOs\n", ngpios);
	else
		dev_err(dev, "Failed to register: ret %d\n", ret);
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

static struct platform_driver mscc_sgpio_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-mscc-sgpio",
		.of_match_table = mscc_sgpio_gpio_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = mscc_sgpio_probe,
};
builtin_platform_driver(mscc_sgpio_pinctrl_driver);
