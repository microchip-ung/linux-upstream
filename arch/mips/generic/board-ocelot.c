// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Microsemi MIPS SoC support
 *
 * Copyright (c) 2017 Microsemi Corporation
 */
#include <asm/machine.h>
#include <asm/prom.h>

struct plat_props {
	phys_addr_t uart_addr;
};

static const struct plat_props ocelot_props = {
	.uart_addr = 0x70100000,
};

static const struct plat_props luton_props = {
	.uart_addr = 0x70100000,
};

static const struct plat_props serval_props = {
	.uart_addr = 0x70100000,
};

static const struct plat_props jr2_props = {
	.uart_addr = 0x70100000,
};

static const struct plat_props *props;

static void __init ocelot_earlyprintk_init(void)
{
	if (props) {
		void __iomem *uart_base;
		uart_base = ioremap_nocache(props->uart_addr, 0x20);
		setup_8250_early_printk_port((unsigned long)uart_base, 2, 50000);
	}
}

static void __init ocelot_late_init(void)
{
	ocelot_earlyprintk_init();
}

static __init const void *ocelot_fixup_fdt(const void *fdt,
					   const void *match_data)
{
	/* This has to be done so late because ioremap needs to work */
	late_time_init = ocelot_late_init;
	props = match_data;

	return fdt;
}

static const struct of_device_id mscc_of_match[] __initconst = {
	{
		.compatible = "mscc,ocelot",
		.data	    = &ocelot_props,
	},{
		.compatible = "mscc,luton",
		.data	    = &luton_props,
	},{
		.compatible = "mscc,serval",
		.data	    = &serval_props,
	},{
		.compatible = "mscc,jr2",
		.data	    = &jr2_props,
	},{
		.compatible = "mscc,serval",
	},{
		.compatible = "mscc,serval",
	},{
		.compatible = "mscc,servalt",
	},{
	}
};

MIPS_MACHINE(ocelot) = {
	.fixup_fdt = ocelot_fixup_fdt,
	.matches = mscc_of_match,
};
