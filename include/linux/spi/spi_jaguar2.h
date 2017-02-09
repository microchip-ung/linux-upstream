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
 * spi_jaguar2 interface to platform code
 *
 */
#ifndef _LINUX_SPI_SPI_JAGUAR2
#define _LINUX_SPI_SPI_JAGUAR2

#include <linux/types.h>
#include <linux/spi/spi.h>


/**
 * struct spi_vcoreiii_platform_data - Data definitions for a SPI-GPIO device.
 *
 * This structure holds information about a GPIO-based SPI device.
 *
 * @no_spi_delay: If true, no delay is done in the lowlevel bitbanging.
 *                Note that doing no delay is not standards compliant,
 *                but it might be needed to speed up transfers on some
 *                slow embedded machines.
 */
struct spi_vcoreiii_platform_data {
	bool no_spi_delay;
};

/**
 * SPI_VCOREIII_PLATDEV_NAME - The platform device name string.
 *
 * The name string that has to be used for platform_device_alloc
 * when allocating a spi-vcoreiii device.
 */
#define SPI_VCOREIII_PLATDEV_NAME	"spi-jaguar2"

#endif /* _LINUX_SPI_SPI_JAGUAR2 */
