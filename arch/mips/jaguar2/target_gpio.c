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
 * VCore-III GPIO
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>

#include <asm/mach-jaguar2/hardware.h>
#include <asm/vcoreiii-gpio.h>

static int jag2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
    return vcoreiii_gpio_get_value(offset);
}

static void jag2_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    vcoreiii_gpio_set_value(offset, value);
}

static int jag2_gpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    vcoreiii_gpio_direction_input(offset);
    return 0;
}

static int jag2_gpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    vcoreiii_gpio_direction_output(offset, value);
    return 0;
}

static struct gpio_chip jag2_gpio_chip = {
    .get = jag2_gpio_get,
    .set = jag2_gpio_set,
    .direction_input = jag2_gpio_dir_in,
    .direction_output = jag2_gpio_dir_out,
    .label = "gpio",
    .base = 0,
    .ngpio = 64,
};

static int jag2_sgpio_get(struct gpio_chip *chip, unsigned int offset)
{
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2_ARCH)
     int ct   = offset / 128;
#endif
     int bit  = (offset % 128) / 32;
     int port = (offset % 128) % 32;
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2_ARCH)
     return !!(readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_INPUT_DATA(ct,bit)) & (1 << port));
#else
     return !!(readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_INPUT_DATA(bit)) & (1 << port));
#endif
}

static void jag2_sgpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2_ARCH)
    int ct   = offset / 128;
#endif
    int bit  = (offset % 128) / 32;
    int port = (offset % 128) % 32;
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2_ARCH)
    u32 val  = readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CFG(ct, port));
#else
    u32 val  = readl(VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CFG(port));
#endif
    u32 mask = (0x7 << (bit*3));
    val = (val & ~mask) | (value << (bit*3));
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2_ARCH)
    writel(val, VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CFG(ct, port));
#else
    writel(val, VTSS_DEVCPU_GCB_SIO_CTRL_SIO_PORT_CFG(port));
#endif
}

static int jag2_sgpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    return 0;
}

static int jag2_sgpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    return 0;
}

static struct gpio_chip jag2_sgpio_chip = {
    .get = jag2_sgpio_get,
    .set = jag2_sgpio_set,
    .direction_input = jag2_sgpio_dir_in,
    .direction_output = jag2_sgpio_dir_out,
    .label = "sgpio",
    .base = 64,
#if defined(CONFIG_VTSS_VCOREIII_JAGUAR2_ARCH)
    .ngpio = 3*4*32,    // 3 controllers, 4 bits, 32 ports
#else
    .ngpio = 1*4*32,    // 1 controller, 4 bits, 32 ports
#endif
};

int __init jag2_gpio_init(void)
{
    int rc;

    if((rc = gpiochip_add(&jag2_gpio_chip)) != 0)
        return rc;
    printk(KERN_WARNING VTSS_TARGET_NAME " registered %d GPIOs\n", jag2_gpio_chip.ngpio);

    if((rc = gpiochip_add(&jag2_sgpio_chip)) != 0)
	    return rc;
    printk(KERN_WARNING VTSS_TARGET_NAME " registered %d SGPIOs\n", jag2_sgpio_chip.ngpio);

    return 0;
}

static int __init vcoreiii_gpio_reserve(void)
{
    /* Standard GPIO's */
    gpio_request(10, "uart_rx");
    gpio_request(11, "uart_tx");
    return 0;
}
late_initcall(vcoreiii_gpio_reserve);
