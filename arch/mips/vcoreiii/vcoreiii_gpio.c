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

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>

#include <asm/mach-vcoreiii/hardware.h>
#include <asm/vcoreiii-gpio.h>

static int vcoreiii_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
    return vcoreiii_gpio_get_value(offset);
}

static void vcoreiii_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
    vcoreiii_gpio_set_value(offset, value);
}

static int vcoreiii_gpio_dir_in(struct gpio_chip *chip, unsigned int offset)
{
    vcoreiii_gpio_direction_input(offset);
    return 0;
}

static int vcoreiii_gpio_dir_out(struct gpio_chip *chip, unsigned int offset, int value)
{
    vcoreiii_gpio_direction_output(offset, value);
    return 0;
}

static struct gpio_chip vcoreiii_gpio_chip = {
    .get = vcoreiii_gpio_get,
    .set = vcoreiii_gpio_set,
    .direction_input = vcoreiii_gpio_dir_in,
    .direction_output = vcoreiii_gpio_dir_out,
    .label = "gpio",
    .base = 0,
    .ngpio = 32,
};

static int __init vcoreiii_gpio_init(void)
{
    int rc;
    if((rc = gpiochip_add(&vcoreiii_gpio_chip)) != 0)
        return rc;
    printk(KERN_WARNING "Board registered %d GPIOs\n", vcoreiii_gpio_chip.ngpio);
    return rc;
}

module_init(vcoreiii_gpio_init);

MODULE_AUTHOR("Lars Povlsen <lpovlsen@vitesse.com>");
MODULE_DESCRIPTION("VCore-III GPIO button driver");
MODULE_LICENSE("GPL");
