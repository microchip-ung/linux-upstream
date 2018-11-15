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
 * VCore-III IRQ Handling
 *
 */
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <asm/irq_cpu.h>
#include <asm/irq_regs.h>
#include <asm/mach-jaguar2/irq.h>
#include <asm/mach-jaguar2/hardware.h>

static void ack_irq_ioc(struct irq_data *data)
{
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_STICKY); /* Ack sticky */
}

static void mask_irq_ioc(struct irq_data *data)
{
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
}

static void mask_ack_irq_ioc(struct irq_data *data)
{
    // Below, the Ack sticky has no effect on level interrupts, because the underlying
    // interrupt hasn't been serviced yet, so it will get set immediately after clearance.
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_ENA_CLR); /* Disable */
    writel(1 << (data->irq - ICPU_IRQ0_BASE), VTSS_ICPU_CFG_INTR_INTR_STICKY); /* Ack sticky */
}

static void unmask_irq_ioc(struct irq_data *data)
{
    unsigned int mask = VTSS_BIT(data->irq - ICPU_IRQ0_BASE);

    // When interrupts are level-triggered, the INTR_STICKY register gets set immediately
    // as long as the underlying source is interrupting. Therefore, when unmasking a
    // level-trigged interrupt we can safely clear the sticky register prior to enabling
    // the interrupt.
    if ((readl(VTSS_ICPU_CFG_INTR_INTR_TRIGGER(0)) & mask) == 0 && (readl(VTSS_ICPU_CFG_INTR_INTR_TRIGGER(1)) & mask) == 0) {
        // Interrupt is level-triggered. Safe to ack it now.
        writel(mask, VTSS_ICPU_CFG_INTR_INTR_STICKY);
    }

    writel(mask, VTSS_ICPU_CFG_INTR_INTR_ENA_SET); /* Enable */
}

static struct irq_chip vcoreiii_irq_ioc = {
    .name = "icpu",
    .irq_ack = ack_irq_ioc,
    .irq_mask = mask_irq_ioc,
    .irq_mask_ack = mask_ack_irq_ioc,
    .irq_unmask = unmask_irq_ioc,
};

void __init vcoreiii_irq_init(void)
{
    u32 i;

    writel(0, VTSS_ICPU_CFG_INTR_INTR_ENA); /* Mask all */
    writel(0xffffffff, VTSS_ICPU_CFG_INTR_INTR_STICKY); /* Ack pending */

    for (i = ICPU_IRQ0_BASE; i <= PCIE_IRQ ; i++)
        irq_set_chip_and_handler(i, &vcoreiii_irq_ioc, handle_level_irq);
    irq_set_chained_handler(INT0_IRQ, handle_simple_irq);
}

static inline int clz(unsigned long x)
{
    __asm__ (
        ".set  push\n"
        ".set  mips32\n"
        "clz   %0, %1\n"
        ".set  pop\n"
        : "=r" (x)
        : "r" (x));

        return x;
}

static inline unsigned int irq_ffs(unsigned int pending)
{
    return -clz(pending) + 31;
}

asmlinkage void plat_irq_dispatch(void)
{
    unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;

    if (pending & STATUSF_IP7) {
        /* CPU timer */
        do_IRQ(TIMER_IRQ);
    } else if (pending & STATUSF_IP2) {
        /* VCoreIII PIC */
        int irq = readl(VTSS_ICPU_CFG_INTR_DST_INTR_IDENT(0));    // OS-controlled IRQ's only
        if (unlikely(irq == 0)) {
            spurious_interrupt();
            return;
        }

        irq = ICPU_IRQ0_BASE + irq_ffs(irq);
        do_IRQ(irq);
    } else if (pending & STATUSF_IP3) {
        /* VCoreIII UIO */
        int irq = readl(VTSS_ICPU_CFG_INTR_DST_INTR_IDENT(1));    // UIO-controlled IRQ's only
        if (unlikely(irq == 0)) {
            spurious_interrupt();
            return;
        }

        do_IRQ(INT1_IRQ);
    } else if (pending & STATUSF_IP0) {
        /* User line 0 */
        do_IRQ(MIPS_SOFTINT0_IRQ);
    } else if (pending & STATUSF_IP1) {
        /* User line 1 */
        do_IRQ(MIPS_SOFTINT1_IRQ);
    } else {
        spurious_interrupt();
    }
}

void __init arch_init_irq(void)
{
    mips_cpu_irq_init();
    // Change INT1_IRQ handler to avoid eoi (unmask)
    // This IRQ is exposed via UIO
    irq_set_handler(INT1_IRQ, handle_level_irq);
}
