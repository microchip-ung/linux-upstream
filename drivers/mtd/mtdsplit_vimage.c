/*
 *  Copyright (C) 2014 Lars Povlsen <lpovlsen@vitesse.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/unaligned.h>

#include "mtdsplit.h"

#define VIMAGE_NR_PARTS		1

static int mtdsplit_parse_vimage(struct mtd_info *master,
                                 struct mtd_partition **pparts,
                                 struct mtd_part_parser_data *data)
{
    size_t rootfs_offset;
    struct mtd_partition *parts;
    int err = mtd_find_rootfs_from(master, master->erasesize, master->size, &rootfs_offset);
    if (err)
        return err;

    parts = kzalloc(VIMAGE_NR_PARTS * sizeof(*parts), GFP_KERNEL);
    if (!parts)
        return -ENOMEM;

    parts[0].name = ROOTFS_PART_NAME;
    parts[0].offset = rootfs_offset;
    parts[0].size = master->size - rootfs_offset;

    *pparts = parts;
    return VIMAGE_NR_PARTS;
}

static struct mtd_part_parser mtdsplit_vimage_parser = {
	.owner = THIS_MODULE,
	.name = "vimage-fw",
	.parse_fn = mtdsplit_parse_vimage,
	.type = MTD_PARSER_TYPE_FIRMWARE,
};

static int __init mtdsplit_vimage_init(void)
{
	register_mtd_parser(&mtdsplit_vimage_parser);

	return 0;
}

subsys_initcall(mtdsplit_vimage_init);
