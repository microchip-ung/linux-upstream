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
 * spi_vcoreiii interface to platform code
 *
 */
#ifndef _LINUX_SPI_SPI_VCOREIII
#define _LINUX_SPI_SPI_VCOREIII

#include <linux/types.h>
#include <linux/spi/spi.h>

#define SPI_VCOREIII_NUM_HW_CS   4
#if defined(CONFIG_VTSS_VCOREIII_MK1) || defined(CONFIG_VTSS_VCOREIII_SERVAL1)
#define SPI_VCOREIII_NUM_GPIO_CS 32
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
#define SPI_VCOREIII_NUM_GPIO_CS 64
#else
#error Unknown platform
#endif

/**
 * struct spi_vcoreiii_platform_data - Data definitions for a SPI-GPIO device.
 *
 * This structure holds information about a GPIO-based SPI device.
 *
 * @chipselect: Controller hook to call before doing a CS. Used to
 * prohibiting simultaneous access on some flawed devices.
 *
 */
struct spi_vcoreiii_platform_data {
    // Platform cs code
    void (*chipselect)(struct spi_device *dev, int on);
};

/*
 * struct spi_vcoreiii_controller_data - custom controller setup.
 *
 * This structure holds custom setup info for specific SPI devices.
 *
 * @mask_lo:
 * @value_lo:
 * @mask_hi:
 * @value_hi:    Set mask, value for using a MUXed CS.
 *
 * @cs_delay:    Wait this many nsecs after applying turning CS "on/off".
 *
 * @chipselect:  Call this custom to control CS. Replaces SPI driver normal CS behaviour.
 * @data:        Can be used for the above @chipselect call.
 *
 */
struct spi_vcoreiii_controller_data {
    u32 mask_lo;    // Low 32 bits of a muxed CS mask
    u32 value_lo;   // Low 32 bits of a muxed CS value
    u32 mask_hi;    // High 32 bits of a muxed CS mask (on platforms that support it)
    u32 value_hi;   // High 32 bits of a muxed CS value (on platforms that support it)
    u32 cs_delay;   // CS "on/off" delay in nsecs.
    // Even more customized CS handler
    void (*chipselect)(struct spi_device *dev, int on);
    void *data;     // For use by above hook
};

/**
 * SPI_VCOREIII_PLATDEV_NAME - The platform device name string.
 *
 * The name string that has to be used for platform_device_alloc
 * when allocating a spi-vcoreiii device.
 */
#define SPI_VCOREIII_PLATDEV_NAME	"spi-vcoreiii"

#endif /* _LINUX_SPI_SPI_VCOREIII */
