/*
 *
 * VCore-III hardware definitions
 *
 * Copyright (C) 2015 Vitesse Semiconductor Inc.
 * Author: Lars Povlsen (lars.povlsen@microsemi.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * VITESSE SEMICONDUCTOR INC SHALL HAVE NO LIABILITY WHATSOEVER OF ANY
 * KIND ARISING OUT OF OR RELATED TO THE PROGRAM OR THE OPEN SOURCE
 * MATERIALS UNDER ANY THEORY OF LIABILITY.
 *
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
