#ifndef DW_SPI_UAPI_HEADER_H
#define DW_SPI_UAPI_HEADER_H

/*
 * Each SPI slave device to work with dw_api controller should
 * has such a structure claiming its working mode (poll or PIO/DMA),
 * which can be save in the "controller_data" member of the
 * struct spi_device.
 */
struct dw_spi_chip {
	u8 poll_mode;	/* 1 for controller polling mode */
	u8 type;	/* SPI/SSP/MicroWire */
	void (*cs_control)(u32 command);
};

#endif /* DW_SPI_UAPI_HEADER_H */
