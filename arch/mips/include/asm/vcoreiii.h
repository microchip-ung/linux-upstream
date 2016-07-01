/* Copyright (c) 2016 Microsemi Corporation

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

#ifndef __ASM_VCOREIII_H
#define __ASM_VCOREIII_H

#if defined(CONFIG_VTSS_VCOREIII_MK1)
#include <asm/mach-vcoreiii/hardware.h>
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
#include <asm/mach-serval/hardware.h>
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
#include <asm/mach-jaguar2/hardware.h>
#else
#error Invalid architecture type
#endif

#ifndef __ASSEMBLER__

extern unsigned long vcoreiii_memmap_start;
extern size_t vcoreiii_memmap_size;

static inline void vcoreiii_io_clr(volatile void __iomem *addr, u32 mask)
{
    writel(readl(addr) & ~mask, addr);
    mmiowb();
}

static inline void vcoreiii_io_set(volatile void __iomem *addr, u32 mask)
{
    writel(readl(addr) | mask, addr);
    mmiowb();
}

static inline void vcoreiii_io_mask_set(volatile void __iomem *addr, u32 clr_mask, u32 set_mask)
{
    u32 val = readl(addr);
    val &= ~clr_mask;
    val |= set_mask;
    writel(val, addr);
    mmiowb();
}

#endif // __ASSEMBLER__

#endif // __ASM_VCOREIII_H
