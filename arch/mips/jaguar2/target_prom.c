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
 *
 * VCore-III PROM functionality
 *
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/serial_reg.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include <asm/fw/fw.h>
#include <linux/initrd.h>
#include <linux/sizes.h>
#include <asm/bootinfo.h>

unsigned long vcoreiii_memmap_start;
size_t vcoreiii_memmap_size;

void __init prom_init(void)
{
	fw_init_cmdline();

#ifdef CONFIG_BLK_DEV_INITRD
	/* Read the initrd address from the firmware environment */
	initrd_start = fw_getenvl("initrd_start");
	if (initrd_start) {
		//strcat(arcs_cmdline, " root=/dev/ram0");
		initrd_start = KSEG0ADDR(initrd_start);
		initrd_end = initrd_start + fw_getenvl("initrd_size");
	}
#endif

	/* Add some random defaults... */
	if ((strstr(arcs_cmdline, "mem=")) == NULL) {
		unsigned long memsize = fw_getenvl("memsize");
		/* Some bootloaders disagree on size in bytes/Mbytes */
		if (memsize > 0 && memsize < SZ_1M) {
			memsize *= SZ_1M;
		}
		if (memsize) {
			unsigned long mmapsize = fw_getenvl("memmap"); /* Note: Always in bytes */
			if (mmapsize) {
				// Reserve the 'memmap' part off the memory end
				memsize -= mmapsize;
				// Setup memmap variables for later UIO driver exposure
				vcoreiii_memmap_start = memsize;
				vcoreiii_memmap_size = mmapsize;
			}
			/* Add directly as memory region */
			add_memory_region(0x00000000, memsize, BOOT_MEM_RAM);
		} else {
			/* Reasonable default */
			strcat(arcs_cmdline, " mem=128M");
		}
	}
	if ((strstr(arcs_cmdline, "console=")) == NULL)
		strcat(arcs_cmdline, " console=ttyS0,115200");
}

void __init prom_free_prom_memory(void)
{
}
