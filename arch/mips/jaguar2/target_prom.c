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

#include <asm/bootinfo.h>

unsigned long vcoreiii_memmap_start;
size_t vcoreiii_memmap_size;

/* This struct is used by Redboot to pass arguments to the kernel */
typedef struct
{
	char *name;
	char *val;
} t_env_var;

struct parmblock {
	t_env_var memsize;
#if 0
	t_env_var memmap;
	t_env_var modetty0;
	t_env_var ethaddr;
	t_env_var env_end;
	char *argv[2];
	char text[0];
#endif
};

static const t_env_var * prom_env;

char * __init prom_getcmdline(void)
{
	return &(arcs_cmdline[0]);
}

static const char __init *prom_getenv(char *name)
{
	if(prom_env) {
		const t_env_var * p;
		for (p = prom_env; p->name != NULL; p++)
			if(strcmp(name, p->name) == 0)
				return p->val;
	}

	return NULL;
}

void __init prom_init(void)
{
    /* Sanity check for defunct bootloader */
    if(fw_arg0 < 10 &&
       (fw_arg1 & 0xFFF00000) == 0x80000000) {
        unsigned int prom_argc = fw_arg0;
        const char ** prom_argv = (const char **) fw_arg1;
        const struct parmblock * const pb = (struct parmblock *) fw_arg2;

        prom_env = &pb->memsize;

        if(prom_argc > 1 && strlen(prom_argv[1]) > 0)
            /* ignore all built-in args if any f/w args given */
            strcpy(arcs_cmdline, prom_argv[1]);
    } else {
        printk(KERN_NOTICE "Invalid kernel arglist - use RedBoot \"exec\" command to boot kernel.\n");
        printk(KERN_NOTICE "Using predefined kernel options.\n");
    }

    /* Add some random defaults... */
    if ((strstr(arcs_cmdline, "console=")) == NULL)
        strcat(arcs_cmdline, " console=ttyS0,115200");
    if ((strstr(arcs_cmdline, "init=")) == NULL)
        strcat(arcs_cmdline, " init=/etc/preinit");
    if ((strstr(arcs_cmdline, "root=")) == NULL)
        strcat(arcs_cmdline, " root=/dev/mtdblock3");
    if ((strstr(arcs_cmdline, "mem=")) == NULL) {
        const char *memopt = prom_getenv("memsize");
        unsigned long memsize;
        if(memopt && (memsize = simple_strtol(memopt, NULL, 16))) {
            const char *mmapopt = prom_getenv("memmap");
            unsigned long mmapsize;
            if(mmapopt && (mmapsize = simple_strtol(mmapopt, NULL, 16))) {
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
}

void __init prom_free_prom_memory(void)
{
}

#ifdef CONFIG_EARLY_PRINTK
void prom_putchar(char c)
{
}
#endif
