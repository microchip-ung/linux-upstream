/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
/*
 * Microchip Sparx5 Ethernet SerDes driver
 *
 * Copyright (c) 2020 Microschip Inc
 */
#ifndef __PHY_ETHERNET_SERDES_H_
#define __PHY_ETHERNET_SERDES_H_

#include <linux/types.h>

enum ethernet_media_type {
	ETH_MEDIA_DEFAULT,
	ETH_MEDIA_SR,
	ETH_MEDIA_DAC,
};

/**
 * struct phy_configure_opts_eth_serdes - Ethernet SerDes This structure is used
 * to represent the configuration state of a Ethernet Serdes PHY.
 * @speed: Speed of the serdes interface in Mbps
 * @media_type: Specifies which media the serdes will be using
 */
struct phy_configure_opts_eth_serdes {
	u32                        speed;
	enum ethernet_media_type   media_type;
};

#endif

