/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * This header provides constants for binding mscc,*-sgpio
 *
 * The first cell in the SGPIO specifier is the GPIO ID. The macros below
 * provide machros for this.
 *
 * The second cell contains standard flag values specified in gpio.h.
 */

#ifndef _DT_BINDINGS_GPIO_MSCC_SGPIO_H
#define _DT_BINDINGS_GPIO_MSCC_SGPIO_H

#include <dt-bindings/gpio/gpio.h>

#define MSCC_SGPIOS_PER_BANK	32
#define MSCC_SGPIO_BANK_DEPTH	4

#define MSCC_SGPIO(port, bit) ((bit * MSCC_SGPIOS_PER_BANK) + port)

#endif
