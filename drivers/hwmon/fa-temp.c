// SPDX-License-Identifier: GPL-2.0-or-later
/* Fireant SoC temperature sensor driver
 *
 * Copyright (C) 2020 Lars Povlsen <lars.povlsen@microchip.com>
 */

#include <linux/bitops.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>

#define TEMP_CTRL		0
#define TEMP_CFG		4
#define  TEMP_CFG_CYCLES	GENMASK(24, 15)
#define  TEMP_CFG_CYCLES_OFF	15
#define  TEMP_CFG_ENA		BIT(0)
#define TEMP_STAT		8
#define  TEMP_STAT_VALID	BIT(12)
#define  TEMP_STAT_TEMP		GENMASK(11, 0)

struct fa_hwmon {
	void __iomem *base;
};

static void fa_temp_enable(struct fa_hwmon *hwmon)
{
	u32 val = readl(hwmon->base + TEMP_CFG);
	u32 clk = 250;

	val &= ~TEMP_CFG_CYCLES;
	val |= (clk << TEMP_CFG_CYCLES_OFF);
	val |= TEMP_CFG_ENA;

	writel(val, hwmon->base + TEMP_CFG);
}

static void fa_temp_disable(void *data)
{
	struct fa_hwmon *hwmon = data;
	u32 val = readl(hwmon->base + TEMP_CFG);

	val &= ~TEMP_CFG_ENA;

	writel(val, hwmon->base + TEMP_CFG);
}

static int fa_read(struct device *dev, enum hwmon_sensor_types type,
		   u32 attr, int channel, long *temp)
{
	struct fa_hwmon *hwmon = dev_get_drvdata(dev);
	int rc = 0, value;
	u32 stat;

	switch (attr) {
	case hwmon_temp_input:
		stat = readl_relaxed(hwmon->base + TEMP_STAT);
		if (stat & TEMP_STAT_VALID) {
			value = (stat & TEMP_STAT_TEMP);
			value = DIV_ROUND_CLOSEST(value * 3522, 4096) - 1094;
			value *= 100;
			*temp = value;
		} else
			rc = -EINVAL;
		break;
	default:
		rc = -EOPNOTSUPP;
	}

	return rc;
}

static umode_t fa_is_visible(const void *_data, enum hwmon_sensor_types type,
			     u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return 0444;
	default:
		return 0;
	}
}

static const struct hwmon_channel_info *fa_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT),
	NULL
};

static const struct hwmon_ops fa_hwmon_ops = {
	.is_visible = fa_is_visible,
	.read = fa_read,
};

static const struct hwmon_chip_info fa_chip_info = {
	.ops = &fa_hwmon_ops,
	.info = fa_info,
};

static int fa_temp_probe(struct platform_device *pdev)
{
	struct device *hwmon_dev;
	struct fa_hwmon *hwmon;
	int err = 0;

	hwmon = devm_kzalloc(&pdev->dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hwmon->base))
		return PTR_ERR(hwmon->base);

	err = devm_add_action(&pdev->dev, fa_temp_disable, hwmon);
	if (err)
		return err;

	fa_temp_enable(hwmon);

	hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
							 "fa_temp",
							 hwmon,
							 &fa_chip_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

const struct of_device_id fa_temp_match[] = {
	{ .compatible = "mscc,fa-temp" },
	{},
};
MODULE_DEVICE_TABLE(of, fa_temp_match);

static struct platform_driver fa_temp_driver = {
	.probe = fa_temp_probe,
	.driver = {
		.name = "fa-temp",
		.of_match_table = fa_temp_match,
	},
};

module_platform_driver(fa_temp_driver);

MODULE_AUTHOR("Lars Povlsen <lars.povlsen@microchip.com>");
MODULE_DESCRIPTION("Fireant SoC temperature sensor driver");
MODULE_LICENSE("GPL");
