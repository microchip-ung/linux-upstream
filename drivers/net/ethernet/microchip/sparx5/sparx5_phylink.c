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
#include "sparx5_port.h"

static void sparx5_phylink_state_print(const char *name, unsigned int portno,
				       const struct phylink_link_state *state)
{
	if (state->interface > 0) {
		pr_debug("%s: portno: %u, ifmode: %s, speed: %d\n",
			 name,
			 portno,
			 phy_modes(state->interface),
			 state->speed);
		pr_debug("  - duplex: %d, pause: %d, link: %d\n",
			 state->duplex,
			 state->pause,
			 state->link);
		pr_debug("  - aneg enable: %u, aneg done: %u\n",
			 state->an_enabled,
			 state->an_complete);
	} else {
		pr_debug("%s: portno: %u, no ifmode", name, portno);
	}
	pr_debug("  - own linkmodes: %*pbl, partner linkmodes: %*pbl\n",
		 __ETHTOOL_LINK_MODE_MASK_NBITS,
		 state->advertising,
		 __ETHTOOL_LINK_MODE_MASK_NBITS,
		 state->lp_advertising);
}

static void sparx5_port_config_print(u32 portno,
				     struct sparx5_port_config *conf)
{
	pr_debug("port: %u: configuration\n", portno);
	pr_debug("  - portmode:   %s\n", phy_modes(conf->portmode));
	pr_debug("  - speed:      %s\n", phy_speed_to_str(conf->speed));
	pr_debug("  - mediatype:  %02u\n", conf->media);
	pr_debug("  - phy_mode:   %s\n", phy_modes(conf->phy_mode));
}

static void sparx5_phylink_validate(struct phylink_config *config,
				    unsigned long *supported,
				    struct phylink_link_state *state)
{
	struct sparx5_port *port = netdev_priv(to_net_dev(config->dev));
	u32 portno = port->portno;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	sparx5_phylink_state_print(__func__, portno, state);

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
	sparx5_phylink_state_print(__func__, portno, state);
}

static bool port_conf_has_changed(struct sparx5_port_config *a, struct sparx5_port_config *b)
{
	if (a->speed != b->speed ||
	    a->portmode != b->portmode ||
	    a->autoneg != b->autoneg ||
	    a->pause != b->pause ||
	    a->media != b->media)
		return true;
	return false;
}

static void sparx5_phylink_mac_config(struct phylink_config *config,
				      unsigned int mode,
				      const struct phylink_link_state *state)
{
	struct sparx5_port *port = netdev_priv(to_net_dev(config->dev));
	struct sparx5_port_config conf;
	int err = 0;

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
			conf.media = PHY_MEDIA_SR;
		else
			conf.media = PHY_MEDIA_DAC;
	}

	if (!port_conf_has_changed(&port->conf, &conf))
		return;

	/* Enable the PCS matching this interface type */
	err = sparx5_port_pcs_set(port->sparx5, port, &conf);
	if (err)
		netdev_err(port->ndev, "port config failed: %d\n", err);

	sparx5_port_config_print(port->portno, &port->conf);
}

static void sparx5_phylink_mac_link_up(struct phylink_config *config,
				       struct phy_device *phy,
				       unsigned int mode,
				       phy_interface_t interface,
				       int speed, int duplex,
				       bool tx_pause, bool rx_pause)
{
	struct sparx5_port *port = netdev_priv(to_net_dev(config->dev));
	struct sparx5_port_config conf;
	int err = 0;

	conf = port->conf;
	conf.duplex = duplex;
	conf.pause = 0;
	conf.pause |= tx_pause ? MLO_PAUSE_TX : 0;
	conf.pause |= rx_pause ? MLO_PAUSE_RX : 0;
	conf.speed = speed;

	/* Configure the port to speed/duplex/pause */
	err = sparx5_port_config(port->sparx5, port, &conf);
	if (err)
		netdev_err(port->ndev, "port config failed: %d\n", err);
}

static void sparx5_phylink_mac_link_state(struct phylink_config *config,
					  struct phylink_link_state *state)
{
	struct sparx5_port *port = netdev_priv(to_net_dev(config->dev));
	struct sparx5_port_status status;

	sparx5_get_port_status(port->sparx5, port, &status);
	state->link = status.link && !status.link_down;
	state->an_complete = status.an_complete;
	state->speed = status.speed;
	state->duplex = status.duplex;
	state->pause = status.pause;
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
