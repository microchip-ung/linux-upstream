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

#include <asm/mach-serval/hardware.h>
#include <asm/vcoreiii-gpio.h>

static int srvl_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
    return vcoreiii_gpio_get_value(offset);
}

static void srvl_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    vcoreiii_gpio_set_value(offset, value);
}

static int srvl_gpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    vcoreiii_gpio_direction_input(offset);
    return 0;
}

static int srvl_gpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    vcoreiii_gpio_direction_output(offset, value);
    return 0;
}

static struct gpio_chip srvl_gpio_chip = {
    .get = srvl_gpio_get,
    .set = srvl_gpio_set,
    .direction_input = srvl_gpio_dir_in,
    .direction_output = srvl_gpio_dir_out,
    .label = "gpio",
    .base = 0,
    .ngpio = 32,
};

static int srvl_sgpio_get(struct gpio_chip *chip, unsigned int offset)
{
    unsigned int ix   = offset / 32;
    unsigned int port = offset % 32;
    return vcoreiii_sgpio_get_value(port, ix);
}

static void srvl_sgpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    unsigned int ix   = offset / 32;
    unsigned int port = offset % 32;
    vcoreiii_sgpio_set_value(port, ix, value);
}

static int srvl_sgpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    return 0;
}

static int srvl_sgpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    return 0;
}

static struct gpio_chip srvl_sgpio_chip = {
    .get = srvl_sgpio_get,
    .set = srvl_sgpio_set,
    .direction_input = srvl_sgpio_dir_in,
    .direction_output = srvl_sgpio_dir_out,
    .label = "sgpio",
    .base = 32,
    .ngpio = 32+32,
};

int __init srvl_gpio_init(void)
{
    int rc;
    //int bit_count = (srvl_sgpio_chip.ngpio / 32); /* Serval1ref */
    //int port;

    if((rc = gpiochip_add(&srvl_gpio_chip)) != 0)
        return rc;
    printk(KERN_WARNING "%s registered %d GPIOs\n", VTSS_TARGET_NAME, srvl_gpio_chip.ngpio);

    if((rc = gpiochip_add(&srvl_sgpio_chip)) != 0)
	    return rc;
    printk(KERN_WARNING "%s registered %d SGPIOs\n", VTSS_TARGET_NAME, srvl_sgpio_chip.ngpio);
    return 0;
}

static int __init vcoreiii_gpio_reserve(void)
{
    /* Standard GPIO's */
#if defined(CONFIG_VTSS_VCOREIII_SERVAL1_CLASSIC)
    vcoreiii_gpio_set_alternate(22, 0); /* VCORE_CFG0-3 */
    vcoreiii_gpio_set_alternate(23, 0);
    vcoreiii_gpio_set_alternate(24, 0);
    vcoreiii_gpio_set_alternate(25, 0);
    /* Standard SGPIO's */
    (void) gpio_request_one(32+ 0+ 7, GPIOF_DIR_IN|GPIOF_EXPORT, "sgpio_pushbutton");
#elif defined(CONFIG_VTSS_VCOREIII_OCELOT)
    vcoreiii_gpio_set_alternate(18, 0); /* VCORE_CFG0-3 */
    vcoreiii_gpio_set_alternate(19, 0);
    vcoreiii_gpio_set_alternate(20, 0);
    vcoreiii_gpio_set_alternate(21, 0);
#else
#error Invalid sub-architecture type
#endif
    return 0;
}
late_initcall(vcoreiii_gpio_reserve);
