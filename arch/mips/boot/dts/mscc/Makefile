# SPDX-License-Identifier: GPL-2.0-only
dtb-$(CONFIG_MSCC_OCELOT)	+= ocelot_pcb123.dtb ocelot_pcb120.dtb
dtb-$(CONFIG_MSCC_LUTON)	+= luton_pcb090.dtb luton_pcb091.dtb

obj-$(CONFIG_BUILTIN_DTB)	+= $(addsuffix .o, $(dtb-y))
