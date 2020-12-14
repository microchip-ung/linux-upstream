// SPDX-License-Identifier: GPL-2.0+
/* Microchip Sparx5 Switch driver
 *
 * Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
 *
 * The Sparx5 Chip Register Model can be browsed at this location:
 * https://github.com/microchip-ung/sparx-5_reginfo
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <net/switchdev.h>
#include <linux/etherdevice.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/reset.h>

#include "sparx5_main_regs.h"
#include "sparx5_main.h"

#define QLIM_WM(fraction) \
	((SPX5_BUFFER_MEMORY / SPX5_BUFFER_CELL_SZ - 100) * (fraction) / 100)
#define IO_RANGES               2

struct initial_port_config {
	u32                       portno;
	struct device_node        *node;
	struct sparx5_port_config conf;
	struct phy                *serdes;
};

struct sparx5_io_resource {
	enum sparx5_target id;
	phys_addr_t        offset;
	int                range;
};

struct sparx5_ram_config {
	void __iomem *init_reg;
	u32  init_val;
};

static const struct sparx5_io_resource sparx5_iomap[] =  {
	{ TARGET_DEV2G5,         0,         0 }, /* 0x610004000: dev2g5_0 */
	{ TARGET_DEV5G,          0x4000,    0 }, /* 0x610008000: dev5g_0 */
	{ TARGET_PCS5G_BR,       0x8000,    0 }, /* 0x61000c000: pcs5g_br_0 */
	{ TARGET_DEV2G5 + 1,     0xc000,    0 }, /* 0x610010000: dev2g5_1 */
	{ TARGET_DEV5G + 1,      0x10000,   0 }, /* 0x610014000: dev5g_1 */
	{ TARGET_PCS5G_BR + 1,   0x14000,   0 }, /* 0x610018000: pcs5g_br_1 */
	{ TARGET_DEV2G5 + 2,     0x18000,   0 }, /* 0x61001c000: dev2g5_2 */
	{ TARGET_DEV5G + 2,      0x1c000,   0 }, /* 0x610020000: dev5g_2 */
	{ TARGET_PCS5G_BR + 2,   0x20000,   0 }, /* 0x610024000: pcs5g_br_2 */
	{ TARGET_DEV2G5 + 6,     0x24000,   0 }, /* 0x610028000: dev2g5_6 */
	{ TARGET_DEV5G + 6,      0x28000,   0 }, /* 0x61002c000: dev5g_6 */
	{ TARGET_PCS5G_BR + 6,   0x2c000,   0 }, /* 0x610030000: pcs5g_br_6 */
	{ TARGET_DEV2G5 + 7,     0x30000,   0 }, /* 0x610034000: dev2g5_7 */
	{ TARGET_DEV5G + 7,      0x34000,   0 }, /* 0x610038000: dev5g_7 */
	{ TARGET_PCS5G_BR + 7,   0x38000,   0 }, /* 0x61003c000: pcs5g_br_7 */
	{ TARGET_DEV2G5 + 8,     0x3c000,   0 }, /* 0x610040000: dev2g5_8 */
	{ TARGET_DEV5G + 8,      0x40000,   0 }, /* 0x610044000: dev5g_8 */
	{ TARGET_PCS5G_BR + 8,   0x44000,   0 }, /* 0x610048000: pcs5g_br_8 */
	{ TARGET_DEV2G5 + 9,     0x48000,   0 }, /* 0x61004c000: dev2g5_9 */
	{ TARGET_DEV5G + 9,      0x4c000,   0 }, /* 0x610050000: dev5g_9 */
	{ TARGET_PCS5G_BR + 9,   0x50000,   0 }, /* 0x610054000: pcs5g_br_9 */
	{ TARGET_DEV2G5 + 10,    0x54000,   0 }, /* 0x610058000: dev2g5_10 */
	{ TARGET_DEV5G + 10,     0x58000,   0 }, /* 0x61005c000: dev5g_10 */
	{ TARGET_PCS5G_BR + 10,  0x5c000,   0 }, /* 0x610060000: pcs5g_br_10 */
	{ TARGET_DEV2G5 + 11,    0x60000,   0 }, /* 0x610064000: dev2g5_11 */
	{ TARGET_DEV5G + 11,     0x64000,   0 }, /* 0x610068000: dev5g_11 */
	{ TARGET_PCS5G_BR + 11,  0x68000,   0 }, /* 0x61006c000: pcs5g_br_11 */
	{ TARGET_DEV2G5 + 12,    0x6c000,   0 }, /* 0x610070000: dev2g5_12 */
	{ TARGET_DEV10G,         0x70000,   0 }, /* 0x610074000: dev10g_0 */
	{ TARGET_PCS10G_BR,      0x74000,   0 }, /* 0x610078000: pcs10g_br_0 */
	{ TARGET_DEV2G5 + 14,    0x78000,   0 }, /* 0x61007c000: dev2g5_14 */
	{ TARGET_DEV10G + 2,     0x7c000,   0 }, /* 0x610080000: dev10g_2 */
	{ TARGET_PCS10G_BR + 2,  0x80000,   0 }, /* 0x610084000: pcs10g_br_2 */
	{ TARGET_DEV2G5 + 15,    0x84000,   0 }, /* 0x610088000: dev2g5_15 */
	{ TARGET_DEV10G + 3,     0x88000,   0 }, /* 0x61008c000: dev10g_3 */
	{ TARGET_PCS10G_BR + 3,  0x8c000,   0 }, /* 0x610090000: pcs10g_br_3 */
	{ TARGET_DEV2G5 + 16,    0x90000,   0 }, /* 0x610094000: dev2g5_16 */
	{ TARGET_DEV2G5 + 17,    0x94000,   0 }, /* 0x610098000: dev2g5_17 */
	{ TARGET_DEV2G5 + 18,    0x98000,   0 }, /* 0x61009c000: dev2g5_18 */
	{ TARGET_DEV2G5 + 19,    0x9c000,   0 }, /* 0x6100a0000: dev2g5_19 */
	{ TARGET_DEV2G5 + 20,    0xa0000,   0 }, /* 0x6100a4000: dev2g5_20 */
	{ TARGET_DEV2G5 + 21,    0xa4000,   0 }, /* 0x6100a8000: dev2g5_21 */
	{ TARGET_DEV2G5 + 22,    0xa8000,   0 }, /* 0x6100ac000: dev2g5_22 */
	{ TARGET_DEV2G5 + 23,    0xac000,   0 }, /* 0x6100b0000: dev2g5_23 */
	{ TARGET_DEV2G5 + 32,    0xb0000,   0 }, /* 0x6100b4000: dev2g5_32 */
	{ TARGET_DEV2G5 + 33,    0xb4000,   0 }, /* 0x6100b8000: dev2g5_33 */
	{ TARGET_DEV2G5 + 34,    0xb8000,   0 }, /* 0x6100bc000: dev2g5_34 */
	{ TARGET_DEV2G5 + 35,    0xbc000,   0 }, /* 0x6100c0000: dev2g5_35 */
	{ TARGET_DEV2G5 + 36,    0xc0000,   0 }, /* 0x6100c4000: dev2g5_36 */
	{ TARGET_DEV2G5 + 37,    0xc4000,   0 }, /* 0x6100c8000: dev2g5_37 */
	{ TARGET_DEV2G5 + 38,    0xc8000,   0 }, /* 0x6100cc000: dev2g5_38 */
	{ TARGET_DEV2G5 + 39,    0xcc000,   0 }, /* 0x6100d0000: dev2g5_39 */
	{ TARGET_DEV2G5 + 40,    0xd0000,   0 }, /* 0x6100d4000: dev2g5_40 */
	{ TARGET_DEV2G5 + 41,    0xd4000,   0 }, /* 0x6100d8000: dev2g5_41 */
	{ TARGET_DEV2G5 + 42,    0xd8000,   0 }, /* 0x6100dc000: dev2g5_42 */
	{ TARGET_DEV2G5 + 43,    0xdc000,   0 }, /* 0x6100e0000: dev2g5_43 */
	{ TARGET_DEV2G5 + 44,    0xe0000,   0 }, /* 0x6100e4000: dev2g5_44 */
	{ TARGET_DEV2G5 + 45,    0xe4000,   0 }, /* 0x6100e8000: dev2g5_45 */
	{ TARGET_DEV2G5 + 46,    0xe8000,   0 }, /* 0x6100ec000: dev2g5_46 */
	{ TARGET_DEV2G5 + 47,    0xec000,   0 }, /* 0x6100f0000: dev2g5_47 */
	{ TARGET_DEV2G5 + 57,    0xf0000,   0 }, /* 0x6100f4000: dev2g5_57 */
	{ TARGET_DEV25G + 1,     0xf4000,   0 }, /* 0x6100f8000: dev25g_1 */
	{ TARGET_PCS25G_BR + 1,  0xf8000,   0 }, /* 0x6100fc000: pcs25g_br_1 */
	{ TARGET_DEV2G5 + 59,    0x100000,  0 }, /* 0x610104000: dev2g5_59 */
	{ TARGET_DEV25G + 3,     0x104000,  0 }, /* 0x610108000: dev25g_3 */
	{ TARGET_PCS25G_BR + 3,  0x108000,  0 }, /* 0x61010c000: pcs25g_br_3 */
	{ TARGET_DEV2G5 + 60,    0x110000,  0 }, /* 0x610114000: dev2g5_60 */
	{ TARGET_DEV25G + 4,     0x114000,  0 }, /* 0x610118000: dev25g_4 */
	{ TARGET_PCS25G_BR + 4,  0x118000,  0 }, /* 0x61011c000: pcs25g_br_4 */
	{ TARGET_DEV2G5 + 64,    0x120000,  0 }, /* 0x610124000: dev2g5_64 */
	{ TARGET_DEV5G + 12,     0x124000,  0 }, /* 0x610128000: dev5g_64 */
	{ TARGET_PCS5G_BR + 12,  0x128000,  0 }, /* 0x61012c000: pcs5g_br_64 */
	{ TARGET_PORT_CONF,      0x12c000,  0 }, /* 0x610130000: port_conf */
	{ TARGET_DEV2G5 + 3,     0x400000,  0 }, /* 0x610404000: dev2g5_3 */
	{ TARGET_DEV5G + 3,      0x404000,  0 }, /* 0x610408000: dev5g_3 */
	{ TARGET_PCS5G_BR + 3,   0x408000,  0 }, /* 0x61040c000: pcs5g_br_3 */
	{ TARGET_DEV2G5 + 4,     0x40c000,  0 }, /* 0x610410000: dev2g5_4 */
	{ TARGET_DEV5G + 4,      0x410000,  0 }, /* 0x610414000: dev5g_4 */
	{ TARGET_PCS5G_BR + 4,   0x414000,  0 }, /* 0x610418000: pcs5g_br_4 */
	{ TARGET_DEV2G5 + 5,     0x418000,  0 }, /* 0x61041c000: dev2g5_5 */
	{ TARGET_DEV5G + 5,      0x41c000,  0 }, /* 0x610420000: dev5g_5 */
	{ TARGET_PCS5G_BR + 5,   0x420000,  0 }, /* 0x610424000: pcs5g_br_5 */
	{ TARGET_DEV2G5 + 13,    0x424000,  0 }, /* 0x610428000: dev2g5_13 */
	{ TARGET_DEV10G + 1,     0x428000,  0 }, /* 0x61042c000: dev10g_1 */
	{ TARGET_PCS10G_BR + 1,  0x42c000,  0 }, /* 0x610430000: pcs10g_br_1 */
	{ TARGET_DEV2G5 + 24,    0x430000,  0 }, /* 0x610434000: dev2g5_24 */
	{ TARGET_DEV2G5 + 25,    0x434000,  0 }, /* 0x610438000: dev2g5_25 */
	{ TARGET_DEV2G5 + 26,    0x438000,  0 }, /* 0x61043c000: dev2g5_26 */
	{ TARGET_DEV2G5 + 27,    0x43c000,  0 }, /* 0x610440000: dev2g5_27 */
	{ TARGET_DEV2G5 + 28,    0x440000,  0 }, /* 0x610444000: dev2g5_28 */
	{ TARGET_DEV2G5 + 29,    0x444000,  0 }, /* 0x610448000: dev2g5_29 */
	{ TARGET_DEV2G5 + 30,    0x448000,  0 }, /* 0x61044c000: dev2g5_30 */
	{ TARGET_DEV2G5 + 31,    0x44c000,  0 }, /* 0x610450000: dev2g5_31 */
	{ TARGET_DEV2G5 + 48,    0x450000,  0 }, /* 0x610454000: dev2g5_48 */
	{ TARGET_DEV10G + 4,     0x454000,  0 }, /* 0x610458000: dev10g_4 */
	{ TARGET_PCS10G_BR + 4,  0x458000,  0 }, /* 0x61045c000: pcs10g_br_4 */
	{ TARGET_DEV2G5 + 49,    0x45c000,  0 }, /* 0x610460000: dev2g5_49 */
	{ TARGET_DEV10G + 5,     0x460000,  0 }, /* 0x610464000: dev10g_5 */
	{ TARGET_PCS10G_BR + 5,  0x464000,  0 }, /* 0x610468000: pcs10g_br_5 */
	{ TARGET_DEV2G5 + 50,    0x468000,  0 }, /* 0x61046c000: dev2g5_50 */
	{ TARGET_DEV10G + 6,     0x46c000,  0 }, /* 0x610470000: dev10g_6 */
	{ TARGET_PCS10G_BR + 6,  0x470000,  0 }, /* 0x610474000: pcs10g_br_6 */
	{ TARGET_DEV2G5 + 51,    0x474000,  0 }, /* 0x610478000: dev2g5_51 */
	{ TARGET_DEV10G + 7,     0x478000,  0 }, /* 0x61047c000: dev10g_7 */
	{ TARGET_PCS10G_BR + 7,  0x47c000,  0 }, /* 0x610480000: pcs10g_br_7 */
	{ TARGET_DEV2G5 + 52,    0x480000,  0 }, /* 0x610484000: dev2g5_52 */
	{ TARGET_DEV10G + 8,     0x484000,  0 }, /* 0x610488000: dev10g_8 */
	{ TARGET_PCS10G_BR + 8,  0x488000,  0 }, /* 0x61048c000: pcs10g_br_8 */
	{ TARGET_DEV2G5 + 53,    0x48c000,  0 }, /* 0x610490000: dev2g5_53 */
	{ TARGET_DEV10G + 9,     0x490000,  0 }, /* 0x610494000: dev10g_9 */
	{ TARGET_PCS10G_BR + 9,  0x494000,  0 }, /* 0x610498000: pcs10g_br_9 */
	{ TARGET_DEV2G5 + 54,    0x498000,  0 }, /* 0x61049c000: dev2g5_54 */
	{ TARGET_DEV10G + 10,    0x49c000,  0 }, /* 0x6104a0000: dev10g_10 */
	{ TARGET_PCS10G_BR + 10, 0x4a0000,  0 }, /* 0x6104a4000: pcs10g_br_10 */
	{ TARGET_DEV2G5 + 55,    0x4a4000,  0 }, /* 0x6104a8000: dev2g5_55 */
	{ TARGET_DEV10G + 11,    0x4a8000,  0 }, /* 0x6104ac000: dev10g_11 */
	{ TARGET_PCS10G_BR + 11, 0x4ac000,  0 }, /* 0x6104b0000: pcs10g_br_11 */
	{ TARGET_DEV2G5 + 56,    0x4b0000,  0 }, /* 0x6104b4000: dev2g5_56 */
	{ TARGET_DEV25G,         0x4b4000,  0 }, /* 0x6104b8000: dev25g_0 */
	{ TARGET_PCS25G_BR,      0x4b8000,  0 }, /* 0x6104bc000: pcs25g_br_0 */
	{ TARGET_DEV2G5 + 58,    0x4c0000,  0 }, /* 0x6104c4000: dev2g5_58 */
	{ TARGET_DEV25G + 2,     0x4c4000,  0 }, /* 0x6104c8000: dev25g_2 */
	{ TARGET_PCS25G_BR + 2,  0x4c8000,  0 }, /* 0x6104cc000: pcs25g_br_2 */
	{ TARGET_DEV2G5 + 61,    0x4d0000,  0 }, /* 0x6104d4000: dev2g5_61 */
	{ TARGET_DEV25G + 5,     0x4d4000,  0 }, /* 0x6104d8000: dev25g_5 */
	{ TARGET_PCS25G_BR + 5,  0x4d8000,  0 }, /* 0x6104dc000: pcs25g_br_5 */
	{ TARGET_DEV2G5 + 62,    0x4e0000,  0 }, /* 0x6104e4000: dev2g5_62 */
	{ TARGET_DEV25G + 6,     0x4e4000,  0 }, /* 0x6104e8000: dev25g_6 */
	{ TARGET_PCS25G_BR + 6,  0x4e8000,  0 }, /* 0x6104ec000: pcs25g_br_6 */
	{ TARGET_DEV2G5 + 63,    0x4f0000,  0 }, /* 0x6104f4000: dev2g5_63 */
	{ TARGET_DEV25G + 7,     0x4f4000,  0 }, /* 0x6104f8000: dev25g_7 */
	{ TARGET_PCS25G_BR + 7,  0x4f8000,  0 }, /* 0x6104fc000: pcs25g_br_7 */
	{ TARGET_DSM,            0x500000,  0 }, /* 0x610504000: dsm */
	{ TARGET_ASM,            0x5fc000,  0 }, /* 0x610600000: asm */
	{ TARGET_GCB,            0x100c000, 1 }, /* 0x611010000: gcb */
	{ TARGET_QS,             0x102c000, 1 }, /* 0x611030000: qs */
	{ TARGET_ANA_ACL,        0x104c000, 1 }, /* 0x611050000: ana_acl */
	{ TARGET_LRN,            0x105c000, 1 }, /* 0x611060000: lrn */
	{ TARGET_VCAP_SUPER,     0x107c000, 1 }, /* 0x611080000: vcap_super */
	{ TARGET_QSYS,           0x109c000, 1 }, /* 0x6110a0000: qsys */
	{ TARGET_QFWD,           0x10ac000, 1 }, /* 0x6110b0000: qfwd */
	{ TARGET_XQS,            0x10bc000, 1 }, /* 0x6110c0000: xqs */
	{ TARGET_CLKGEN,         0x10fc000, 1 }, /* 0x611100000: clkgen */
	{ TARGET_ANA_AC_POL,     0x11fc000, 1 }, /* 0x611200000: ana_ac_pol */
	{ TARGET_QRES,           0x127c000, 1 }, /* 0x611280000: qres */
	{ TARGET_EACL,           0x12bc000, 1 }, /* 0x6112c0000: eacl */
	{ TARGET_ANA_CL,         0x13fc000, 1 }, /* 0x611400000: ana_cl */
	{ TARGET_ANA_L3,         0x147c000, 1 }, /* 0x611480000: ana_l3 */
	{ TARGET_HSCH,           0x157c000, 1 }, /* 0x611580000: hsch */
	{ TARGET_REW,            0x15fc000, 1 }, /* 0x611600000: rew */
	{ TARGET_ANA_L2,         0x17fc000, 1 }, /* 0x611800000: ana_l2 */
	{ TARGET_ANA_AC,         0x18fc000, 1 }, /* 0x611900000: ana_ac */
	{ TARGET_VOP,            0x19fc000, 1 }, /* 0x611a00000: vop */
};

static int sparx5_create_targets(struct sparx5 *sparx5)
{
	struct resource *iores[IO_RANGES];
	void __iomem *iomem[IO_RANGES];
	void __iomem *begin[IO_RANGES];
	int range_id[IO_RANGES];
	int idx, jdx;

	for (idx = 0, jdx = 0; jdx < ARRAY_SIZE(sparx5_iomap); jdx++) {
		const struct sparx5_io_resource *iomap = &sparx5_iomap[jdx];

		if (idx == iomap->range) {
			range_id[idx] = jdx;
			idx++;
		}
	}
	for (idx = 0; idx < IO_RANGES; idx++) {
		iores[idx] = platform_get_resource(sparx5->pdev, IORESOURCE_MEM,
						   idx);
		iomem[idx] = devm_ioremap(sparx5->dev,
					  iores[idx]->start,
					  iores[idx]->end - iores[idx]->start
					  + 1);
		if (IS_ERR(iomem[idx])) {
			dev_err(sparx5->dev, "Unable to get switch registers: %s\n",
				iores[idx]->name);
			return PTR_ERR(iomem[idx]);
		}
		begin[idx] = iomem[idx] - sparx5_iomap[range_id[idx]].offset;
	}
	for (jdx = 0; jdx < ARRAY_SIZE(sparx5_iomap); jdx++) {
		const struct sparx5_io_resource *iomap = &sparx5_iomap[jdx];

		sparx5->regs[iomap->id] = begin[iomap->range] + iomap->offset;
	}
	return 0;
}

static int sparx5_create_port(struct sparx5 *sparx5,
			      struct initial_port_config *config)
{
	struct sparx5_port *spx5_port;

	/* netdev creation to be added in later patches */
	spx5_port = devm_kzalloc(sparx5->dev, sizeof(*spx5_port), GFP_KERNEL);
	spx5_port->of_node = config->node;
	spx5_port->serdes = config->serdes;
	spx5_port->pvid = NULL_VID;
	spx5_port->signd_internal = true;
	spx5_port->signd_active_high = true;
	spx5_port->signd_enable = true;
	spx5_port->max_vlan_tags = SPX5_PORT_MAX_TAGS_NONE;
	spx5_port->vlan_type = SPX5_VLAN_PORT_TYPE_UNAWARE;
	spx5_port->custom_etype = 0x8880; /* Vitesse */

	/* PHYLINK support to be added in later patches */

	return 0;
}

static int sparx5_init_ram(struct sparx5 *s5)
{
	const struct sparx5_ram_config spx5_ram_cfg[] = {
		{spx5_reg_get(s5, ANA_AC_STAT_RESET), ANA_AC_STAT_RESET_RESET},
		{spx5_reg_get(s5, ASM_STAT_CFG), ASM_STAT_CFG_STAT_CNT_CLR_SHOT},
		{spx5_reg_get(s5, QSYS_RAM_INIT), QSYS_RAM_INIT_RAM_INIT},
		{spx5_reg_get(s5, REW_RAM_INIT), QSYS_RAM_INIT_RAM_INIT},
		{spx5_reg_get(s5, VOP_RAM_INIT), QSYS_RAM_INIT_RAM_INIT},
		{spx5_reg_get(s5, ANA_AC_RAM_INIT), QSYS_RAM_INIT_RAM_INIT},
		{spx5_reg_get(s5, ASM_RAM_INIT), QSYS_RAM_INIT_RAM_INIT},
		{spx5_reg_get(s5, EACL_RAM_INIT), QSYS_RAM_INIT_RAM_INIT},
		{spx5_reg_get(s5, VCAP_SUPER_RAM_INIT), QSYS_RAM_INIT_RAM_INIT},
		{spx5_reg_get(s5, DSM_RAM_INIT), QSYS_RAM_INIT_RAM_INIT}
	};
	const struct sparx5_ram_config *cfg;
	u32 value, pending, jdx, idx;

	for (jdx = 0; jdx < 10; jdx++) {
		pending = ARRAY_SIZE(spx5_ram_cfg);
		for (idx = 0; idx < ARRAY_SIZE(spx5_ram_cfg); idx++) {
			cfg = &spx5_ram_cfg[idx];
			if (jdx == 0) {
				writel(cfg->init_val, cfg->init_reg);
			} else {
				value = readl(cfg->init_reg);
				if ((value & cfg->init_val) != cfg->init_val)
					pending--;
			}
		}
		if (!pending)
			break;
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
	}

	if (pending > 0) {
		/* Still initializing, should be complete in
		 * less than 1ms
		 */
		dev_err(s5->dev, "Memory initialization error\n");
		return -EINVAL;
	}
	return 0;
}

static int sparx5_init_switchcore(struct sparx5 *sparx5)
{
	u32 value;

	spx5_rmw(EACL_POL_EACL_CFG_EACL_FORCE_INIT_SET(1),
		 EACL_POL_EACL_CFG_EACL_FORCE_INIT,
		 sparx5,
		 EACL_POL_EACL_CFG);

	spx5_rmw(EACL_POL_EACL_CFG_EACL_FORCE_INIT_SET(0),
		 EACL_POL_EACL_CFG_EACL_FORCE_INIT,
		 sparx5,
		 EACL_POL_EACL_CFG);

	/* Initialize memories, if not done already */
	value = spx5_rd(sparx5, HSCH_RESET_CFG);
	if (!(value & HSCH_RESET_CFG_CORE_ENA))
		sparx5_init_ram(sparx5);

	/* Reset counters */
	spx5_wr(ANA_AC_STAT_RESET_RESET_SET(1), sparx5, ANA_AC_STAT_RESET);
	spx5_wr(ASM_STAT_CFG_STAT_CNT_CLR_SHOT_SET(1), sparx5, ASM_STAT_CFG);

	/* Injection/Extraction to be added in later patches */
	/* Enable switch-core and queue system */
	spx5_wr(HSCH_RESET_CFG_CORE_ENA_SET(1), sparx5, HSCH_RESET_CFG);

	return 0;
}

static int sparx5_init_coreclock(struct sparx5 *sparx5)
{
	enum sparx5_core_clockfreq freq = sparx5->coreclock;
	u32 clk_div, clk_period, pol_upd_int, idx;

	/* Verify if core clock frequency is supported on target.
	 * If 'VTSS_CORE_CLOCK_DEFAULT' then the highest supported
	 * freq. is used
	 */
	switch (sparx5->target_ct) {
	case SPX5_TARGET_CT_7546:
		if (sparx5->coreclock == SPX5_CORE_CLOCK_DEFAULT)
			freq = SPX5_CORE_CLOCK_250MHZ;
		else if (sparx5->coreclock != SPX5_CORE_CLOCK_250MHZ)
			freq = 0; /* Not supported */
		break;
	case SPX5_TARGET_CT_7549:
	case SPX5_TARGET_CT_7552:
	case SPX5_TARGET_CT_7556:
		if (sparx5->coreclock == SPX5_CORE_CLOCK_DEFAULT)
			freq = SPX5_CORE_CLOCK_500MHZ;
		else if (sparx5->coreclock != SPX5_CORE_CLOCK_500MHZ)
			freq = 0; /* Not supported */
		break;
	case SPX5_TARGET_CT_7558:
	case SPX5_TARGET_CT_7558TSN:
		if (sparx5->coreclock == SPX5_CORE_CLOCK_DEFAULT)
			freq = SPX5_CORE_CLOCK_625MHZ;
		else if (sparx5->coreclock != SPX5_CORE_CLOCK_625MHZ)
			freq = 0; /* Not supported */
		break;
	case SPX5_TARGET_CT_7546TSN:
		if (sparx5->coreclock == SPX5_CORE_CLOCK_DEFAULT)
			freq = SPX5_CORE_CLOCK_625MHZ;
		break;
	case SPX5_TARGET_CT_7549TSN:
	case SPX5_TARGET_CT_7552TSN:
	case SPX5_TARGET_CT_7556TSN:
		if (sparx5->coreclock == SPX5_CORE_CLOCK_DEFAULT)
			freq = SPX5_CORE_CLOCK_625MHZ;
		else if (sparx5->coreclock == SPX5_CORE_CLOCK_250MHZ)
			freq = 0; /* Not supported */
		break;
	default:
		dev_err(sparx5->dev, "Target (%#04x) not supported\n",
			sparx5->target_ct);
		return -ENODEV;
	}

	switch (freq) {
	case SPX5_CORE_CLOCK_250MHZ:
		clk_div = 10;
		pol_upd_int = 312;
		break;
	case SPX5_CORE_CLOCK_500MHZ:
		clk_div = 5;
		pol_upd_int = 624;
		break;
	case SPX5_CORE_CLOCK_625MHZ:
		clk_div = 4;
		pol_upd_int = 780;
		break;
	default:
		dev_err(sparx5->dev, "%s: %d cloreclock not supported on (%#04x)\n",
			__func__,
			sparx5->coreclock, sparx5->target_ct);
		return -EINVAL;
	}

	/* Update state with chosen frequency */
	sparx5->coreclock = freq;

	/* Configure the LCPLL */
	spx5_rmw(CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_CLK_DIV_SET(clk_div) |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_PRE_DIV_SET(0) |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_ROT_DIR_SET(0) |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_ROT_SEL_SET(0) |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_ROT_ENA_SET(0) |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_CLK_ENA_SET(1),
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_CLK_DIV |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_PRE_DIV |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_ROT_DIR |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_ROT_SEL |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_ROT_ENA |
		 CLKGEN_LCPLL1_CORE_CLK_CFG_CORE_CLK_ENA,
		 sparx5,
		 CLKGEN_LCPLL1_CORE_CLK_CFG);

	clk_period = sparx5_clk_period(freq);

	spx5_rmw(HSCH_SYS_CLK_PER_SYS_CLK_PER_100PS_SET(clk_period / 100),
		 HSCH_SYS_CLK_PER_SYS_CLK_PER_100PS,
		 sparx5,
		 HSCH_SYS_CLK_PER);

	spx5_rmw(ANA_AC_POL_BDLB_DLB_CTRL_CLK_PERIOD_01NS_SET(clk_period / 100),
		 ANA_AC_POL_BDLB_DLB_CTRL_CLK_PERIOD_01NS,
		 sparx5,
		 ANA_AC_POL_BDLB_DLB_CTRL);

	spx5_rmw(ANA_AC_POL_SLB_DLB_CTRL_CLK_PERIOD_01NS_SET(clk_period / 100),
		 ANA_AC_POL_SLB_DLB_CTRL_CLK_PERIOD_01NS,
		 sparx5,
		 ANA_AC_POL_SLB_DLB_CTRL);

	spx5_rmw(LRN_AUTOAGE_CFG_1_CLK_PERIOD_01NS_SET(clk_period / 100),
		 LRN_AUTOAGE_CFG_1_CLK_PERIOD_01NS,
		 sparx5,
		 LRN_AUTOAGE_CFG_1);

	for (idx = 0; idx < 3; idx++) {
		spx5_rmw(GCB_SIO_CLOCK_SYS_CLK_PERIOD_SET(clk_period / 100),
			 GCB_SIO_CLOCK_SYS_CLK_PERIOD,
			 sparx5,
			 GCB_SIO_CLOCK(idx));
	}

	spx5_rmw(HSCH_TAS_STATEMACHINE_CFG_REVISIT_DLY_SET
		 ((256 * 1000) / clk_period),
		 HSCH_TAS_STATEMACHINE_CFG_REVISIT_DLY,
		 sparx5,
		 HSCH_TAS_STATEMACHINE_CFG);

	spx5_rmw(ANA_AC_POL_POL_UPD_INT_CFG_POL_UPD_INT_SET(pol_upd_int),
		 ANA_AC_POL_POL_UPD_INT_CFG_POL_UPD_INT,
		 sparx5,
		 ANA_AC_POL_POL_UPD_INT_CFG);

	return 0;
}

static int sparx5_qlim_set(struct sparx5 *sparx5)
{
	u32 res, dp, prio;

	for (res = 0; res < 2; res++) {
		for (prio = 0; prio < 8; prio++)
			spx5_wr(0xFFF, sparx5,
				QRES_RES_CFG(prio + 630 + res * 1024));

		for (dp = 0; dp < 4; dp++)
			spx5_wr(0xFFF, sparx5,
				QRES_RES_CFG(dp + 638 + res * 1024));
	}

	/* Set 80,90,95,100% of memory size for top watermarks */
	spx5_wr(QLIM_WM(80), sparx5, XQS_QLIMIT_SHR_QLIM_CFG(0));
	spx5_wr(QLIM_WM(90), sparx5, XQS_QLIMIT_SHR_CTOP_CFG(0));
	spx5_wr(QLIM_WM(95), sparx5, XQS_QLIMIT_SHR_ATOP_CFG(0));
	spx5_wr(QLIM_WM(100), sparx5, XQS_QLIMIT_SHR_TOP_CFG(0));

	return 0;
}

/* Some boards needs to map the SGPIO for signal detect explicitly to the
 * port module
 */
static void sparx5_board_init(struct sparx5 *sparx5)
{
	int idx;

	if (!sparx5->sd_sgpio_remapping)
		return;

	/* Enable SGPIO Signal Detect remapping */
	spx5_rmw(GCB_HW_SGPIO_SD_CFG_SD_MAP_SEL,
		 GCB_HW_SGPIO_SD_CFG_SD_MAP_SEL,
		 sparx5,
		 GCB_HW_SGPIO_SD_CFG);

	/* Refer to LOS SGPIO */
	for (idx = 0; idx < SPX5_PORTS; idx++) {
		if (sparx5->ports[idx]) {
			if (sparx5->ports[idx]->conf.sd_sgpio != ~0) {
				spx5_wr(sparx5->ports[idx]->conf.sd_sgpio,
					sparx5,
					GCB_HW_SGPIO_TO_SD_MAP_CFG(idx));
			}
		}
	}
}

static int sparx5_start(struct sparx5 *sparx5)
{
	u32 idx;

	if (sparx5_create_targets(sparx5))
		return -ENODEV;

	/* Read chip ID to check CPU interface */
	sparx5->chip_id = spx5_rd(sparx5, GCB_CHIP_ID);

	sparx5->target_ct = (enum spx5_target_chiptype)
		GCB_CHIP_ID_PART_ID_GET(sparx5->chip_id);

	/* Initialize Switchcore and internal RAMs */
	if (sparx5_init_switchcore(sparx5)) {
		dev_err(sparx5->dev, "Switchcore initialization error\n");
		return -EINVAL;
	}

	/* Initialize the LC-PLL (core clock) and set affected registers */
	if (sparx5_init_coreclock(sparx5)) {
		dev_err(sparx5->dev, "LC-PLL initialization error\n");
		return -EINVAL;
	}

	/* Setup own UPSIDs */
	for (idx = 0; idx < 3; idx++) {
		spx5_wr(idx, sparx5, ANA_AC_OWN_UPSID(idx));
		spx5_wr(idx, sparx5, ANA_CL_OWN_UPSID(idx));
		spx5_wr(idx, sparx5, ANA_L2_OWN_UPSID(idx));
		spx5_wr(idx, sparx5, REW_OWN_UPSID(idx));
	}

	/* Enable CPU ports */
	for (idx = SPX5_PORTS; idx < SPX5_PORTS_ALL; idx++) {
		spx5_rmw(QFWD_SWITCH_PORT_MODE_PORT_ENA_SET(1),
			 QFWD_SWITCH_PORT_MODE_PORT_ENA,
			 sparx5,
			 QFWD_SWITCH_PORT_MODE(idx));
	}

	/* Forwarding masks to be added in later patches */
	/* CPU copy CPU pgids */
	spx5_wr(ANA_AC_PGID_MISC_CFG_PGID_CPU_COPY_ENA_SET(1),
		sparx5, ANA_AC_PGID_MISC_CFG(PGID_CPU));
	spx5_wr(ANA_AC_PGID_MISC_CFG_PGID_CPU_COPY_ENA_SET(1),
		sparx5, ANA_AC_PGID_MISC_CFG(PGID_BCAST));

	/* Recalc injected frame FCS */
	for (idx = SPX5_PORT_CPU_0; idx <= SPX5_PORT_CPU_1; idx++)
		spx5_rmw(ANA_CL_FILTER_CTRL_FORCE_FCS_UPDATE_ENA_SET(1),
			 ANA_CL_FILTER_CTRL_FORCE_FCS_UPDATE_ENA,
			 sparx5, ANA_CL_FILTER_CTRL(idx));

	/* MAC/VLAN support to be added in later patches */
	/* Enable queue limitation watermarks */
	sparx5_qlim_set(sparx5);

	/* netdev and resource calendar support to be added in later patches */

	sparx5_board_init(sparx5);

	return 0;
}

static int mchp_sparx5_probe(struct platform_device *pdev)
{
	struct initial_port_config *configs, *config;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ports, *portnp;
	struct sparx5 *sparx5;
	int idx = 0, err = 0;
	const u8 *mac_addr;

	if (!np && !pdev->dev.platform_data)
		return -ENODEV;

	sparx5 = devm_kzalloc(&pdev->dev, sizeof(*sparx5), GFP_KERNEL);
	if (!sparx5)
		return -ENOMEM;

	platform_set_drvdata(pdev, sparx5);
	sparx5->pdev = pdev;
	sparx5->dev = &pdev->dev;

	sparx5->reset = devm_reset_control_get_shared(&pdev->dev, "switch");
	if (IS_ERR(sparx5->reset)) {
		dev_warn(sparx5->dev, "Could not obtain reset control: %ld\n",
			 PTR_ERR(sparx5->reset));
		sparx5->reset = NULL;
	} else {
		reset_control_reset(sparx5->reset);
	}

	/* Default values, some from DT */
	sparx5->coreclock = SPX5_CORE_CLOCK_DEFAULT;

	ports = of_get_child_by_name(np, "ethernet-ports");
	if (!ports) {
		dev_err(sparx5->dev, "no ethernet-ports child node found\n");
		return -ENODEV;
	}
	sparx5->port_count = of_get_child_count(ports);

	configs = kcalloc(sparx5->port_count,
			  sizeof(struct initial_port_config), GFP_KERNEL);
	if (!configs)
		return -ENOMEM;

	for_each_available_child_of_node(ports, portnp) {
		struct sparx5_port_config *conf;
		struct phy *serdes;
		u32 portno;

		err = of_property_read_u32(portnp, "reg", &portno);
		if (err) {
			dev_err(sparx5->dev, "port reg property error\n");
			continue;
		}
		config = &configs[idx];
		conf = &config->conf;
		err = of_get_phy_mode(portnp, &conf->phy_mode);
		if (err) {
			dev_err(sparx5->dev, "port %u: missing phy-mode\n",
				portno);
			continue;
		}
		err = of_property_read_u32(portnp, "bandwidth",
					   &conf->bandwidth);
		if (err) {
			dev_err(sparx5->dev, "port %u: missing bandwidth\n",
				portno);
			continue;
		}
		err = of_property_read_u32(portnp, "sd_sgpio", &conf->sd_sgpio);
		if (err)
			conf->sd_sgpio = ~0;
		else
			sparx5->sd_sgpio_remapping = true;
		serdes = devm_of_phy_get(sparx5->dev, portnp, NULL);
		if (IS_ERR(serdes)) {
			err = PTR_ERR(serdes);
			if (err != -EPROBE_DEFER)
				dev_err(sparx5->dev,
					"port %u: missing serdes\n",
					portno);
			goto cleanup_config;
		}
		config->portno = portno;
		config->node = portnp;
		config->serdes = serdes;

		conf->media_type = ETH_MEDIA_DAC;
		conf->serdes_reset = true;
		conf->portmode = conf->phy_mode;
		if (of_find_property(portnp, "sfp", NULL)) {
			conf->has_sfp = true;
			conf->power_down = true;
		}
		idx++;
	}

	err = sparx5_create_targets(sparx5);
	if (err)
		goto cleanup_config;

	mac_addr = of_get_mac_address(np);
	if (IS_ERR_OR_NULL(mac_addr)) {
		dev_info(sparx5->dev, "MAC addr was not set, use random MAC\n");
		eth_random_addr(sparx5->base_mac);
		sparx5->base_mac[5] = 0;
	} else {
		ether_addr_copy(sparx5->base_mac, mac_addr);
	}

	/* Inj/Xtr IRQ support to be added in later patches */
	/* Read chip ID to check CPU interface */
	sparx5->chip_id = spx5_rd(sparx5, GCB_CHIP_ID);

	sparx5->target_ct = (enum spx5_target_chiptype)
		GCB_CHIP_ID_PART_ID_GET(sparx5->chip_id);

	/* Initialize Switchcore and internal RAMs */
	if (sparx5_init_switchcore(sparx5)) {
		dev_err(sparx5->dev, "Switchcore initialization error\n");
		goto cleanup_config;
	}

	/* Initialize the LC-PLL (core clock) and set affected registers */
	if (sparx5_init_coreclock(sparx5)) {
		dev_err(sparx5->dev, "LC-PLL initialization error\n");
		goto cleanup_config;
	}

	for (idx = 0; idx < sparx5->port_count; ++idx) {
		config = &configs[idx];
		if (!config->node)
			continue;

		err = sparx5_create_port(sparx5, config);
		if (err) {
			dev_err(sparx5->dev, "port create error\n");
			goto cleanup_ports;
		}
	}

	if (sparx5_start(sparx5)) {
		dev_err(sparx5->dev, "Start failed\n");
		goto cleanup_ports;
	}

	kfree(configs);
	return err;

cleanup_ports:
	/* Port cleanup to be added in later patches */
cleanup_config:
	kfree(configs);
	return err;
}

static const struct of_device_id mchp_sparx5_match[] = {
	{ .compatible = "microchip,sparx5-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, mchp_sparx5_match);

static struct platform_driver mchp_sparx5_driver = {
	.probe = mchp_sparx5_probe,
	.driver = {
		.name = "sparx5-switch",
		.of_match_table = mchp_sparx5_match,
	},
};

module_platform_driver(mchp_sparx5_driver);

MODULE_DESCRIPTION("Microchip Sparx5 switch driver");
MODULE_AUTHOR("Steen Hegelund <steen.hegelund@microchip.com>");
MODULE_LICENSE("Dual MIT/GPL");
