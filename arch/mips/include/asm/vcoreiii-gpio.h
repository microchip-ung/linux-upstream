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

#ifndef __ASM_VCOREIII_GPIO_H
#define __ASM_VCOREIII_GPIO_H

#include <asm/bug.h>
#include <asm/vcoreiii.h>

#define VCOREIII_GPIO_MODE_NORMAL 0
#define VCOREIII_GPIO_MODE_ALT1   1
#define VCOREIII_GPIO_MODE_ALT2   2
#define VCOREIII_GPIO_MODE_ALT3   3

//#define WANT_DEBUG 1
#if defined(WANT_DEBUG)
#define DEBUG(arg...) printk(KERN_ERR arg)
#else
#define DEBUG(arg...) /* Go away */
#endif

static inline void vcoreiii_gpio_set_mode(int gpio, int mode)
{
    int i;
    if (gpio < 32) {
        u32 mask = VTSS_BIT(gpio);
        // Serialize mode bits into GPIO_ALT(x) registers at proper position
        for (i = 0; i < 2; i++, mode >>= 1) {
            if(mode & 1) {
                vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(i), mask);
            } else {
                vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT(i), mask);
            }
        }
    } else {
#if defined(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT1)
        u32 mask = VTSS_BIT(gpio - 32);
        // Serialize mode bits into GPIO_ALT1(x) registers at proper position
        for (i = 0; i < 2; i++, mode >>= 1) {
            if(mode & 1) {
                vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT1(i), mask);
            } else {
                vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_ALT1(i), mask);
            }
        }
#else
        BUG();
#endif
    }
}

static inline void vcoreiii_gpio_set_alternate(int gpio, int is_alternate)
{
    vcoreiii_gpio_set_mode(gpio, is_alternate ? VCOREIII_GPIO_MODE_ALT1 : VCOREIII_GPIO_MODE_NORMAL);
}

static inline void vcoreiii_gpio_direction_input(int gpio)
{
    vcoreiii_gpio_set_mode(gpio, VCOREIII_GPIO_MODE_NORMAL);
    if (gpio < 32) {
        u32 mask = VTSS_BIT(gpio);
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, mask);
        DEBUG("core: direction_input GPIO %d, OE = 0x%08x\n", gpio, readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OE));
    } else {
#if defined(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1)
        u32 mask = VTSS_BIT(gpio - 32);
        vcoreiii_io_clr(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1, mask);
        DEBUG("core: direction_input GPIO %d, OE1 = 0x%08x\n", gpio, readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1));
#else
        BUG();
#endif
    }
}

static inline void vcoreiii_gpio_set_value(unsigned int gpio, int value)
{
    if (gpio < 32) {
        u32 mask = VTSS_BIT(gpio);
        if(value)
            writel(mask, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_SET);
        else
            writel(mask, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_CLR);
        DEBUG("core: Set GPIO %d = %d, OUT = 0x%08x\n", gpio, value, readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT));
    } else {
#if defined (VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_SET1)
        u32 mask = VTSS_BIT(gpio - 32);
        if(value)
            writel(mask, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_SET1);
        else
            writel(mask, VTSS_DEVCPU_GCB_GPIO_GPIO_OUT_CLR1);
        DEBUG("core: Set GPIO %d = %d, OUT1 = 0x%08x\n", gpio, value, readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OUT1));
#else
        BUG();
#endif
    }
    mmiowb();
}

static inline void vcoreiii_gpio_direction_output(unsigned int gpio, int value)
{
    // First value (avoid glitches)
    vcoreiii_gpio_set_value(gpio, value);
    // Then OE
    if (gpio < 32) {
        u32 mask = VTSS_BIT(gpio);
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE, mask);
        DEBUG("core:  GPIO %d, OE = 0x%08x\n", gpio, readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OE));
    } else {
#if defined(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1)
        u32 mask = VTSS_BIT(gpio - 32);
        vcoreiii_io_set(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1, mask);
        DEBUG("core: direction_input GPIO %d, OE1 = 0x%08x\n", gpio, readl(VTSS_DEVCPU_GCB_GPIO_GPIO_OE1));
#else
        BUG();
#endif
    }
    // NB: This puposely does *NOT mess withe the mode settings!
}

static inline int vcoreiii_gpio_get_value(unsigned int gpio)
{
    if (gpio < 32) {
        return !!(readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN) & VTSS_BIT(gpio));
    } else {
#if defined(VTSS_DEVCPU_GCB_GPIO_GPIO_IN1)
        return !!(readl(VTSS_DEVCPU_GCB_GPIO_GPIO_IN1) & VTSS_BIT(gpio - 32));
#else
        BUG();
#endif
    }
}

#if defined(CONFIG_VTSS_VCOREIII_SERVAL1)
static inline int vcoreiii_sgpio_get_value(unsigned int port, unsigned int bit)
{
    if (port < 32 && bit < 4) {
        return !!(readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_INPUT_DATA(bit)) & (1 << port));
    } else {
        BUG();
    }
}
#endif /* defined(CONFIG_VTSS_VCOREIII_SERVAL1) */

#if defined(CONFIG_VTSS_VCOREIII_SERVAL1)
static inline void vcoreiii_sgpio_set_value(unsigned int port, unsigned int bit, unsigned int value)
{
    if (port < 32 && bit < 4 && value < 8) {
        u32 val  = readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CONFIG(port));
        u32 mask = VTSS_F_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CONFIG_BIT_SOURCE(0x7 << (bit*3));
        val = (val & ~mask) | VTSS_F_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CONFIG_BIT_SOURCE(value << (bit*3));
        writel(val, VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CONFIG(port));
    } else {
        BUG();
    }
    mmiowb();
}
#endif /* defined(CONFIG_VTSS_VCOREIII_SERVAL1) */

#endif /* __ASM_VCOREIII_GPIO_H */
