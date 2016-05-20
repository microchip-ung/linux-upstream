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
 * Hardware access header file.
 *
 */

#ifndef __VCOREIII_HARDWARE_H
#define __VCOREIII_HARDWARE_H

#include <asm/vcoreiii.h>

/* We use virtual memory to map the registers */
#define VTSS_IO_OFFSET1(offset) (map_base_switch + offset)
#define VTSS_IO_OFFSET2(offset) (map_base_cpu + offset)

/* We just want the addresses from VTSS_IOREG() */
#define VTSS_IOREG(t,o)	((void __iomem *)VTSS_IOADDR(t,o))

#define VTSS_IO_PI_REGION(x) (0x50000000 + (0x4000000*x))

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
#include <vtss_jaguar_core_regs.h>
#define VTSS_DEVCPU_PI_PI_PI_MODE            VTSS_IOREG(VTSS_TO_DEVCPU_PI,0x3)
#elif defined(CONFIG_VTSS_VCOREIII_LUTON26)
#include <vtss_luton26_core_regs.h>
#else
#error Invalid sub-architecture type
#endif

#define VCOREIII_CPU_CLOCK 416666666              /* 416MHz */
#define VCOREIII_AHB_CLOCK (VCOREIII_CPU_CLOCK/2) /* 208MHz - half of the CPU freq */

#ifndef __ASSEMBLER__

extern void __iomem *map_base_switch;
extern void __iomem *map_base_cpu;
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
extern void __iomem *map_base_slv_switch;
#endif  /* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
void slv_writel(u32 val, volatile u32 __iomem *addr);
u32  slv_readl(volatile u32 __iomem *addr);

static inline void vcoreiii_io_clr_slv(volatile void __iomem *addr, u32 mask)
{
    slv_writel(slv_readl(addr) & ~mask, addr);
}

static inline void vcoreiii_io_set_slv(volatile void __iomem *addr, u32 mask)
{
    slv_writel(slv_readl(addr) | mask, addr);
}

static inline void vcoreiii_io_mask_set_slv(volatile void __iomem *addr, u32 clr_mask, u32 set_mask)
{
    u32 val = slv_readl(addr);
    val &= ~clr_mask;
    val |= set_mask;
    slv_writel(val, addr);
}

#endif	/* CONFIG_VTSS_VCOREIII_JAGUAR_DUAL */

u32 vcoreiii_reg_to_phys(void __iomem *addr);

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR_DUAL)
void vcoreiii_gpio_set_alternate_slv(int gpio, int is_alternate);
void vcoreiii_gpio_set_input_slv(int gpio, int is_input);
#endif

#endif // __ASSEMBLER__

#endif /* __VCOREIII_HARDWARE_H */
