/*
 * spi_jaguar2 interface to platform code
 *
 * Copyright (c) 2014 Wenxi Jin
 *
 * based on spi_gpio.c
 *  Copyright (c) 2008 Piotr Skamruk
 *  Copyright (c) 2008 Michael Buesch
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
