/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * This header provides constants for binding the Microchip SGPIO
 * driver.
 *
 */

#ifndef _DT_BINDINGS_GPIO_MCHP_SGPIO_H
#define _DT_BINDINGS_GPIO_MCHP_SGPIO_H

#include <dt-bindings/gpio/gpio.h>

/* mchp-sgpio specific pin type defines */
#undef PIN_OUTPUT
#undef PIN_INPUT
#define PIN_OUTPUT	0
#define PIN_INPUT	1

#endif
