/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include "mt29f_spinand.h"

#define STATUS_ECC_MASK_MACRONIX        0x30
#define STATUS_ECC_ERROR_MACRONIX       0x20

static void wb25_set_addr(struct spi_device *spi_nand, struct spinand_cmd *cmd, u32 page_id, u16 offset)
{
	switch (cmd->cmd) {
	case CMD_READ:			/* 13 - 'Page Data Read' into buffer */
	case CMD_PROG_PAGE_EXC:
	case CMD_ERASE_BLK:
		cmd->n_addr = 3;
		cmd->addr[0] = (u8) (page_id >> 16);
		cmd->addr[1] = (u8) (page_id >> 8);
		cmd->addr[2] = (u8) (page_id);
		break;
	case CMD_READ_RDM:		/* 03 - Read (out) data from cache buffer */
		cmd->n_addr = 3;
		cmd->addr[0] = (u8) (offset >> 8);
		cmd->addr[1] = (u8) (offset);
		cmd->addr[2] = (u8) 0xff;
		break;
	case CMD_PROG_PAGE_CLRCACHE:	/* 02 - Load Program Data into cache buffer - 16 bit offset */
		cmd->n_addr = 2;
		cmd->addr[0] = (u8) (offset >> 8);
		cmd->addr[1] = (u8) (offset);
		break;
	default:
		dev_err(&spi_nand->dev, "Unknown command: 0x%02x\n", cmd->cmd);
	}
}

static int wb25_verify_ecc(u8 status)
{
	int ecc_status = (status & STATUS_ECC_MASK_MACRONIX);
	if ((ecc_status == STATUS_ECC_ERROR_MACRONIX) ||
	    (ecc_status == STATUS_ECC_MASK_MACRONIX))
		return SPINAND_ECC_ERROR;
	else if (ecc_status)
		return SPINAND_ECC_CORRECTED;
	return SPINAND_ECC_OK;
}

#ifdef CONFIG_MTD_SPINAND_ONDIEECC
static int wb_spinand_ooblayout_64_ecc(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	if (section > 3)
		return -ERANGE;

	oobregion->offset = (section * 16) + 8;
	oobregion->length = 8;

	return 0;
}

static int wb_spinand_ooblayout_64_free(struct mtd_info *mtd, int section,
					struct mtd_oob_region *oobregion)
{
	if (section > 3)
		return -ERANGE;

	oobregion->offset = (section * 16) + 2;
	oobregion->length = 6;

	return 0;
}

static const struct mtd_ooblayout_ops wb_spinand_oob_64_ops = {
	.ecc = wb_spinand_ooblayout_64_ecc,
	.free = wb_spinand_ooblayout_64_free,
};
#endif

const struct spinand_ops wb25_ops = {
	wb25_set_addr,
	wb25_verify_ecc,
#ifdef CONFIG_MTD_SPINAND_ONDIEECC
	&wb_spinand_oob_64_ops,
#endif
};

static void mx35_set_addr(struct spi_device *spi_nand, struct spinand_cmd *cmd, u32 page_id, u16 offset)
{
	switch (cmd->cmd) {
	case CMD_READ:			/* 13 - 'Page Data Read' into buffer */
	case CMD_PROG_PAGE_EXC:
	case CMD_ERASE_BLK:
		cmd->n_addr = 3;
		cmd->addr[0] = (u8) (page_id >> 16);
		cmd->addr[1] = (u8) (page_id >> 8);
		cmd->addr[2] = (u8) (page_id);
		break;
	case CMD_READ_RDM:		/* 03 - Read (out) data from cache buffer */
		cmd->n_addr = 3;
		cmd->addr[0] = (u8)((offset & 0xff00) >> 8);
		cmd->addr[0] |= (u8)(((page_id >> 6) & 0x1) << 4);
		cmd->addr[1] = (u8)(offset & 0x00ff);
		cmd->addr[2] = (u8)(0xff);
		break;
	case CMD_PROG_PAGE_CLRCACHE:	/* 02 - Load Program Data into cache buffer - 16 bit offset */
		cmd->n_addr = 2;
		cmd->addr[0] = (u8)((offset & 0xff00) >> 8);
		cmd->addr[0] |= (u8)(((page_id >> 6) & 0x1) << 4);
		cmd->addr[1] = (u8)(offset & 0x00ff);
		break;
	default:
		dev_err(&spi_nand->dev, "Unknown command: 0x%02x\n", cmd->cmd);
	}
}

const struct spinand_ops mx35_ops = {
	mx35_set_addr,
	wb25_verify_ecc,
#ifdef CONFIG_MTD_SPINAND_ONDIEECC
	&wb_spinand_oob_64_ops,
#endif
};
