/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
/ {
	images {
	        ramdisk {
	        	description = "ramdisk";
			data = /incbin/("../../../rootfs.squashfs");
			type = "ramdisk";
			arch = "mips";
			os = "linux";
			compression = "none";
	        };
		fdt@luton_pcb090 {
			description = "MSCC Luton PCB090 Device Tree";
			data = /incbin/("boot/dts/mscc/luton_pcb090.dtb");
			type = "flat_dt";
			arch = "mips";
			compression = "none";
			hash@0 {
				algo = "sha1";
			};
		};
	};

	configurations {
		conf@luton_pcb090 {
			description = "Luton Linux kernel";
			kernel = "kernel@0";
                        ramdisk = "ramdisk";
			fdt = "fdt@luton_pcb090";
		};
	};
};
