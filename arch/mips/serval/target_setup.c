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
 * VCore-III System initialization
 *
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/ioport.h>	/* for struct resource */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/reboot.h>

#include <asm/time.h>
#include <asm/idle.h>
#include <asm/reboot.h>

#include <asm/mach-serval/hardware.h>

/* No other usable initialization hook than this ...  */
extern void (*late_time_init)(void);

void __iomem *map_base_switch;
void __iomem *map_base_cpu;

EXPORT_SYMBOL(map_base_switch);
EXPORT_SYMBOL(map_base_cpu);

static struct resource vcoreiii_reserved_resources[] = {
        {
		.name   = "Switch registers",
		.start  = VTSS_IO_ORIGIN1_OFFSET,
		.end    = VTSS_IO_ORIGIN1_OFFSET + VTSS_IO_ORIGIN1_SIZE - 1,
		.flags  = IORESOURCE_MEM
        },
        {
		.name   = "CPU Registers",
		.start  = VTSS_IO_ORIGIN2_OFFSET,
		.end    = VTSS_IO_ORIGIN2_OFFSET + VTSS_IO_ORIGIN2_SIZE - 1,
		.flags  = IORESOURCE_MEM
        },
};

unsigned int get_c0_compare_int(void)
{
	return TIMER_IRQ;
}

void __init plat_time_init(void)
{
	mips_hpt_frequency = VCOREIII_AHB_CLOCK;
}

const char *get_system_type(void)
{
	return "Vitesse VCore-III Serval";
}

static void vcoreiii_machine_restart(char *command)
{
	u32 val;

        do_kernel_restart(command);

#if defined(CONFIG_VTSS_VCOREIII_SERVAL1_CLASSIC)
#define CORE_RST_PROTECT VTSS_F_ICPU_CFG_CPU_SYSTEM_CTRL_RESET_CORE_RST_PROTECT
#define SOFT_CHIP_RST    VTSS_F_DEVCPU_GCB_CHIP_REGS_SOFT_RST_SOFT_CHIP_RST
#else
// Never variants of Serval-1 have another register macro layout
#define CORE_RST_PROTECT VTSS_M_ICPU_CFG_CPU_SYSTEM_CTRL_RESET_CORE_RST_PROTECT
#define SOFT_CHIP_RST    VTSS_M_DEVCPU_GCB_CHIP_REGS_SOFT_RST_SOFT_CHIP_RST
#endif

	// Make sure VCore is NOT protected from reset
	val = readl(VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_RESET);
	val &= ~CORE_RST_PROTECT; // Clear it
	writel(val, VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_RESET);

	writel(SOFT_CHIP_RST, VTSS_DEVCPU_GCB_CHIP_REGS_SOFT_RST);

	while (1)
		cpu_wait();

#undef CORE_RST_PROTECT
#undef SOFT_CHIP_RST
}

u32 vcoreiii_reg_to_phys(void __iomem *addr)
{
    u32 offset;
    if(addr > map_base_switch && (offset = (addr - map_base_switch)) < VTSS_IO_ORIGIN1_SIZE) {
        return VTSS_IO_ORIGIN1_OFFSET + offset;
    } else if (addr > map_base_cpu && (offset = (addr - map_base_cpu)) < VTSS_IO_ORIGIN2_SIZE) {
        return VTSS_IO_ORIGIN2_OFFSET + offset;
    } else {
        printk("Invalid register address, %p reg1 %p reg2 %p\n",
               addr, map_base_switch, map_base_cpu);
        BUG();
        return -1;
    }
}
EXPORT_SYMBOL(vcoreiii_reg_to_phys);


/* Not only time init but that's what the hook it's called through is named */
static void __init vcoreiii_late_time_init(void)
{
	extern void __init vcoreiii_irq_init(void);
        extern void __init srvl_gpio_init(void);

	map_base_switch = ioremap(vcoreiii_reserved_resources[0].start,
				  vcoreiii_reserved_resources[0].end -
				  vcoreiii_reserved_resources[0].start + 1);
	if(!map_base_switch)
                printk(KERN_ERR "Unable to ioremap switch register space\n");

	map_base_cpu = ioremap(vcoreiii_reserved_resources[1].start,
			       vcoreiii_reserved_resources[1].end -
			       vcoreiii_reserved_resources[1].start + 1);
	if(!map_base_cpu)
                printk(KERN_ERR "Unable to ioremap cpu register space\n");

	/* Hook chained IRQs */
	vcoreiii_irq_init();

        /* Register GPIO's */
        srvl_gpio_init();
}

void __init plat_mem_setup(void)
{
	int i;

	/* Callbacks for halt, restart */
	_machine_restart = vcoreiii_machine_restart;
	_machine_halt    = NULL;

	/* Set up initialization hooks */
	late_time_init = vcoreiii_late_time_init;

        for (i = 0; i < ARRAY_SIZE(vcoreiii_reserved_resources); i++)
                request_resource(&ioport_resource, vcoreiii_reserved_resources + i);
}
