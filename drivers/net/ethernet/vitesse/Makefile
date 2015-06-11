#
# Makefile for the Vitesse Semiconductor network device drivers.
#

ccflags-$(CONFIG_VC3FDMA) := -Idrivers/net/ethernet/vitesse/ufdma/include

obj-$(CONFIG_VC3FDMA) += vtss_vc3fdma.o

vtss_vc3fdma-y := vc3fdma.o ufdma/src/ail/ufdma.o
vtss_vc3fdma-$(CONFIG_VTSS_VCOREIII_LUTON26) += ufdma/src/luton26/ufdma_luton26.o
vtss_vc3fdma-$(CONFIG_VTSS_VCOREIII_JAGUAR)  += 
vtss_vc3fdma-$(CONFIG_VTSS_VCOREIII_SERVAL1) += ufdma/src/serval/ufdma_serval.o
vtss_vc3fdma-$(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY) += ufdma/src/jaguar2/ufdma_jaguar2.o

obj-$(CONFIG_VC3MUX)  += vtss_if_mux.o

vtss_if_mux-y := vtss_if_mux_main.o vtss_if_mux_dev.o vtss_if_mux_netlink.o

