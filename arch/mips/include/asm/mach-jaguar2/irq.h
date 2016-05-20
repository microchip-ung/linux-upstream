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

#ifndef __JAGUAR2_IRQ_H
#define __JAGUAR2_IRQ_H

#define NR_IRQS 64

/*
 * CPU core Interrupt Numbers
 */
#define MIPS_CPU_IRQ_BASE       0
#define MIPS_CPU_IRQ(x)         (MIPS_CPU_IRQ_BASE + (x))
#define MIPS_SOFTINT0_IRQ       MIPS_CPU_IRQ(0)
#define MIPS_SOFTINT1_IRQ       MIPS_CPU_IRQ(1)
#define INT0_IRQ                MIPS_CPU_IRQ(2)
#define INT1_IRQ                MIPS_CPU_IRQ(3)
#define INT2_IRQ                MIPS_CPU_IRQ(4)
#define INT3_IRQ                MIPS_CPU_IRQ(5)
#define INT4_IRQ                MIPS_CPU_IRQ(6)
#define TIMER_IRQ               MIPS_CPU_IRQ(7)

/*
 * ICPU IRQ0 Interrupt Numbers
 */
#define ICPU_IRQ0_BASE          (TIMER_IRQ + 1) /* 8 */
#define ICPU_IRQ0(x)            (ICPU_IRQ0_BASE + (x))

#define DEV_ALL_IRQ       ICPU_IRQ0(0)
#define EXT0_IRQ          ICPU_IRQ0(1)
#define EXT1_IRQ          ICPU_IRQ0(2)
#define TIMER0_IRQ        ICPU_IRQ0(3)
#define TIMER1_IRQ        ICPU_IRQ0(4)
#define TIMER2_IRQ        ICPU_IRQ0(5)
#define UART_IRQ          ICPU_IRQ0(6)
#define UART2_IRQ         ICPU_IRQ0(7)
#define TWI_IRQ           ICPU_IRQ0(8)
#define TWI2_IRQ          ICPU_IRQ0(9)
#define SIMC_IRQ          ICPU_IRQ0(10)
#define SW0_IRQ           ICPU_IRQ0(11)
#define SW1_IRQ           ICPU_IRQ0(12)
#define SGPIO_IRQ         ICPU_IRQ0(13)
#if defined(CONFIG_VTSS_VCOREIII_SERVALT)
#define GPIO_IRQ          ICPU_IRQ0(14)
#define MIIM0_INTR_IRQ    ICPU_IRQ0(15)
#define MIIM1_INTR_IRQ    ICPU_IRQ0(16)
#define FDMA_IRQ          ICPU_IRQ0(17)
#define ANA_IRQ           ICPU_IRQ0(18)
#define PTP_RDY_IRQ       ICPU_IRQ0(19)
#define PTP_SYNC_IRQ      ICPU_IRQ0(20)
#define INTEGRITY_IRQ     ICPU_IRQ0(21)
#define XTR_RDY_IRQ       ICPU_IRQ0(22)
#define INJ_RDY_IRQ       ICPU_IRQ0(23)
#define PCIE_IRQ          ICPU_IRQ0(24)
#define OAM_VOP_IRQ       ICPU_IRQ0(25)
#define DPLL_IRQ          ICPU_IRQ0(26)
#else
#define SGPIO1_IRQ        ICPU_IRQ0(14)
#define SGPIO2_IRQ        ICPU_IRQ0(15)
#define GPIO_IRQ          ICPU_IRQ0(16)
#define MIIM0_INTR_IRQ    ICPU_IRQ0(17)
#define MIIM1_INTR_IRQ    ICPU_IRQ0(18)
#define MIIM2_INTR_IRQ    ICPU_IRQ0(19)
#define FDMA_IRQ          ICPU_IRQ0(20)
#define ANA_IRQ           ICPU_IRQ0(21)
#define PTP_RDY_IRQ       ICPU_IRQ0(22)
#define PTP_SYNC_IRQ      ICPU_IRQ0(23)
#define INTEGRITY_IRQ     ICPU_IRQ0(24)
#define XTR_RDY_IRQ       ICPU_IRQ0(25)
#define INJ_RDY_IRQ       ICPU_IRQ0(26)
#define PCIE_IRQ          ICPU_IRQ0(27)
#define OAM_VOP_IRQ       ICPU_IRQ0(28)
#endif

#include_next <irq.h>

#endif /* __JAGUAR2_IRQ_H */
