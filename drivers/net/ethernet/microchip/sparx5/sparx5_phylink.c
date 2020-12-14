// SPDX-License-Identifier: GPL-2.0+
/* Microchip Sparx5 Switch driver
 *
 * Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
 */

#include <linux/module.h>
#include <linux/phylink.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/sfp.h>

#include "sparx5_main_regs.h"
#include "sparx5_main.h"

static void sparx5_phylink_validate(struct phylink_config *config,
				    unsigned long *supported,
				    struct phylink_link_state *state)
{
	struct sparx5_port *port = netdev_priv(to_net_dev(config->dev));
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	phylink_set(mask, Autoneg);
	phylink_set_port_modes(mask);
	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_NA:
		if (port->conf.bandwidth == SPEED_25000 ||
		    port->conf.bandwidth == SPEED_10000) {
			phylink_set(mask, 5000baseT_Full);
			phylink_set(mask, 10000baseT_Full);
			phylink_set(mask, 10000baseCR_Full);
			phylink_set(mask, 10000baseSR_Full);
			phylink_set(mask, 10000baseLR_Full);
			phylink_set(mask, 10000baseLRM_Full);
			phylink_set(mask, 10000baseER_Full);
		}
		if (port->conf.bandwidth == SPEED_25000) {
			phylink_set(mask, 25000baseCR_Full);
			phylink_set(mask, 25000baseSR_Full);
		}
		if (state->interface != PHY_INTERFACE_MODE_NA)
			break;
		fallthrough;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		phylink_set(mask, 10baseT_Half);
		phylink_set(mask, 10baseT_Full);
		phylink_set(mask, 100baseT_Half);
		phylink_set(mask, 100baseT_Full);
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseX_Full);
		if (state->interface != PHY_INTERFACE_MODE_NA)
			break;
		fallthrough;
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_2500BASEX:
		if (state->interface != PHY_INTERFACE_MODE_2500BASEX) {
			phylink_set(mask, 1000baseT_Full);
			phylink_set(mask, 1000baseX_Full);
		}
		if (state->interface == PHY_INTERFACE_MODE_2500BASEX ||
		    state->interface == PHY_INTERFACE_MODE_NA) {
			phylink_set(mask, 2500baseT_Full);
			phylink_set(mask, 2500baseX_Full);
		}
		break;
	default:
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		return;
	}
	bitmap_and(supported, supported, mask, __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
}

static bool port_conf_has_changed(struct sparx5_port_config *a, struct sparx5_port_config *b)
{
	if (a->speed != b->speed ||
	    a->portmode != b->portmode ||
	    a->autoneg != b->autoneg ||
	    a->pause != b->pause ||
	    a->media_type != b->media_type)
		return true;
	return false;
}

static void sparx5_phylink_mac_config(struct phylink_config *config,
				      unsigned int mode,
				      const struct phylink_link_state *state)
{
	struct sparx5_port *port = netdev_priv(to_net_dev(config->dev));
	struct sparx5_port_config conf;

	conf = port->conf;
	conf.power_down = false;
	conf.portmode = state->interface;
	conf.speed = state->speed;
	conf.autoneg = state->an_enabled;
	conf.pause = state->pause;

	if (state->interface == PHY_INTERFACE_MODE_10GBASER) {
		if (state->speed == SPEED_UNKNOWN) {
			/* When a SFP is plugged in we use capabilities to
			 * default to the highest supported speed
			 */
			if (phylink_test(state->advertising, 25000baseSR_Full) ||
			    phylink_test(state->advertising, 25000baseCR_Full))
				conf.speed = SPEED_25000;
			else if (state->interface == PHY_INTERFACE_MODE_10GBASER)
				conf.speed = SPEED_10000;
		} else if (state->speed == SPEED_2500) {
			conf.portmode = PHY_INTERFACE_MODE_2500BASEX;
		} else if (state->speed == SPEED_1000) {
			conf.portmode = PHY_INTERFACE_MODE_1000BASEX;
		}

		if (phylink_test(state->advertising, FIBRE))
			conf.media_type = ETH_MEDIA_SR;
		else
			conf.media_type = ETH_MEDIA_DAC;
	}

	if (!port_conf_has_changed(&port->conf, &conf))
		return;
}

static void sparx5_phylink_mac_link_up(struct phylink_config *config,
				       struct phy_device *phy,
				       unsigned int mode,
				       phy_interface_t interface,
				       int speed, int duplex,
				       bool tx_pause, bool rx_pause)
{
	/* Currently not used */
}

static void sparx5_phylink_mac_link_state(struct phylink_config *config,
					  struct phylink_link_state *state)
{
	state->link = true;
	state->an_complete = true;
	state->speed = SPEED_1000;
	state->duplex = true;
	state->pause = MLO_PAUSE_AN;
}

static void sparx5_phylink_mac_aneg_restart(struct phylink_config *config)
{
	/* Currently not used */
}

static void sparx5_phylink_mac_link_down(struct phylink_config *config,
					 unsigned int mode,
					 phy_interface_t interface)
{
	/* Currently not used */
}

const struct phylink_mac_ops sparx5_phylink_mac_ops = {
	.validate = sparx5_phylink_validate,
	.mac_pcs_get_state = sparx5_phylink_mac_link_state,
	.mac_config = sparx5_phylink_mac_config,
	.mac_an_restart = sparx5_phylink_mac_aneg_restart,
	.mac_link_down = sparx5_phylink_mac_link_down,
	.mac_link_up = sparx5_phylink_mac_link_up,
};
