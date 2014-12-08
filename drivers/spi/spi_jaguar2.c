/*
 * Hardware SPI master controller driver for Jaguar2
 *
 * Copyright (c) 2014 Wenxi Jin.
 *
 * based on spi-dw.c
 *  Copyright (c) 2009, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "spi_jaguar2.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define START_STATE	((void *)0)
#define RUNNING_STATE	((void *)1)
#define DONE_STATE	((void *)2)
#define ERROR_STATE	((void *)-1)

#define QUEUE_RUNNING	0
#define QUEUE_STOPPED	1

#define MRST_SPI_DEASSERT	0
#define MRST_SPI_ASSERT		1

/* Slave spi_dev related */
struct chip_data {
	u16 cr0;
	u8 cs;			/* chip select pin */
	u8 n_bytes;		/* current is a 1/2/4 byte op */
	u8 tmode;		/* TR/TO/RO/EEPROM */
	u8 type;		/* SPI/SSP/MicroWire */

	u8 poll_mode;		/* 1 means use poll mode */

	/* u32 dma_width; */
	u32 rx_threshold;
	u32 tx_threshold;
	/* u8 enable_dma; */
	u8 bits_per_word;
	u16 clk_div;		/* baud rate divider */
	u32 speed_hz;		/* baud rate */
	void (*cs_control)(u32 command);
};

#ifdef CONFIG_DEBUG_FS
#define SPI_REGS_BUFSIZE	1024
static ssize_t  spi_show_regs(struct file *file, char __user *user_buf,
                              size_t count, loff_t *ppos)
{
    struct jaguar2_spi *jaguar2s;
    char *buf;
    u32 len = 0;
    ssize_t ret;

    dws = file->private_data;

    buf = kzalloc(SPI_REGS_BUFSIZE, GFP_KERNEL);
    if (!buf)
        return 0;

    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "MRST SPI0 registers:\n");
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "=================================\n");
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "CTRL0: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_CTRLR0));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "CTRL1: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_CTRLR1));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "SSIENR: \t0x%08x\n", readl(VTSS_SIMC_SIMC_SSIENR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "SER: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_SER));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "BAUDR: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_BAUDR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "TXFTLR: \t0x%08x\n", readl(VTSS_SIMC_SIMC_TXFTLR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "RXFTLR: \t0x%08x\n", readl(VTSS_SIMC_SIMC_RXFTLR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "TXFLR: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_TXFLR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "RXFLR: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_RXFLR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "SR: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_SR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "IMR: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_IMR));
    len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
                    "ISR: \t\t0x%08x\n", readl(VTSS_SIMC_SIMC_ISR));
                    "=================================\n");

    ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
    kfree(buf);
    return ret;
}

static const struct file_operations mrst_spi_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= spi_show_regs,
	.llseek		= default_llseek,
};

static int mrst_spi_debugfs_init(struct jaguar2_spi *jaguar2s)
{
	jaguar2s->debugfs = debugfs_create_dir("mrst_spi", NULL);
	if (!jaguar2s->debugfs)
		return -ENOMEM;

	debugfs_create_file("registers", S_IFREG | S_IRUGO,
		jaguar2s->debugfs, (void *)jaguar2s, &mrst_spi_regs_ops);
	return 0;
}

static void mrst_spi_debugfs_remove(struct jaguar2_spi *jaguar2s)
{
	if (jaguar2s->debugfs)
		debugfs_remove_recursive(jaguar2s->debugfs);
}

#else
static inline int mrst_spi_debugfs_init(struct jaguar2_spi *jaguar2s)
{
	return 0;
}

static inline void mrst_spi_debugfs_remove(struct jaguar2_spi *jaguar2s)
{
}
#endif /* CONFIG_DEBUG_FS */

/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max(struct jaguar2_spi *jaguar2s)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (jaguar2s->tx_end - jaguar2s->tx) / jaguar2s->n_bytes;
	tx_room = jaguar2s->fifo_len - (u32)readl(VTSS_SIMC_SIMC_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (jaguar2s->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap =  ((jaguar2s->rx_end - jaguar2s->rx) - (jaguar2s->tx_end - jaguar2s->tx))
			/ jaguar2s->n_bytes;

	return min3(tx_left, tx_room, (u32) (jaguar2s->fifo_len - rxtx_gap));
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct jaguar2_spi *jaguar2s)
{
	u32 rx_left = (jaguar2s->rx_end - jaguar2s->rx) / jaguar2s->n_bytes;

	return min(rx_left, (u32)readl(VTSS_SIMC_SIMC_RXFLR));
}

static void jaguar2_writer(struct jaguar2_spi *jaguar2s)
{
	u32 max = tx_max(jaguar2s);
	u16 txw = 0;
        struct jaguar2_spi *sp;
        u32 sw_mode = readl(VTSS_ICPU_CFG_SPI_MST_SW_MODE);

        /*
         * SPI master controller automatically deasserts chip select when tx fifo is empty
         * Workaround: continuous toggle the chip select of SPI boot controller since the
         * final chip select is AND gate between these two.
         * Note: it is only valid on jaguar2 platform. ServalT is using multiplexer
         * instead of AND gate.
         */
        sw_mode |= VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE(1);
        writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        
        sw_mode |= VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS_OE(VTSS_BIT(jaguar2s->cur_chip->cs)) |
                VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS(VTSS_BIT(jaguar2s->cur_chip->cs));
        writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        
	while (max--) {
		/* Set the tx word if the transfer's original "tx" is not null */
		if (jaguar2s->tx_end - jaguar2s->len) {
			if (jaguar2s->n_bytes == 1)
				txw = *(u8 *)(jaguar2s->tx);
			else
				txw = *(u16 *)(jaguar2s->tx);
		}
                
                sp = spidev_to_sg(jaguar2s->master);
		sp->bb_cur = VTSS_F_SIMC_SIMC_DR_DR(txw);
                writel(sp->bb_cur, VTSS_SIMC_SIMC_DR(0));
		jaguar2s->tx += jaguar2s->n_bytes;
	}
}

static void jaguar2_reader(struct jaguar2_spi *jaguar2s)
{
	u32 max = rx_max(jaguar2s);
	u16 rxw;

	while (max--) {
            rxw = readl(VTSS_SIMC_SIMC_DR(0));
		/* Care rx only if the transfer's original "rx" is not null */
		if (jaguar2s->rx_end - jaguar2s->len) {
			if (jaguar2s->n_bytes == 1)
				*(u8 *)(jaguar2s->rx) = rxw;
			else
				*(u16 *)(jaguar2s->rx) = rxw;
		}
		jaguar2s->rx += jaguar2s->n_bytes;
	}
}

static void *next_transfer(struct jaguar2_spi *jaguar2s)
{
	struct spi_message *msg = jaguar2s->cur_msg;
	struct spi_transfer *trans = jaguar2s->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		jaguar2s->cur_transfer =
			list_entry(trans->transfer_list.next,
					struct spi_transfer,
					transfer_list);
		return RUNNING_STATE;
	} else
		return DONE_STATE;
}

/* Caller already set message->status; dma and pio irqs are blocked */
static void giveback(struct jaguar2_spi *jaguar2s)
{
	struct spi_transfer *last_transfer;
	unsigned long flags;
	struct spi_message *msg;
        u32 sw_mode = readl(VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        
	spin_lock_irqsave(&jaguar2s->lock, flags);
	msg = jaguar2s->cur_msg;
	jaguar2s->cur_msg = NULL;
	jaguar2s->cur_transfer = NULL;
	jaguar2s->prev_chip = jaguar2s->cur_chip;
	jaguar2s->cur_chip = NULL;
	queue_work(jaguar2s->workqueue, &jaguar2s->pump_messages);
	spin_unlock_irqrestore(&jaguar2s->lock, flags);

	last_transfer = list_entry(msg->transfers.prev,
					struct spi_transfer,
					transfer_list);

	if (!last_transfer->cs_change && jaguar2s->cs_control)
		jaguar2s->cs_control(MRST_SPI_DEASSERT);

        /*  untoggle chip select */
        sw_mode |= VTSS_F_ICPU_CFG_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE(1);
        writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        
        sw_mode &= ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS_OE &
                ~VTSS_M_ICPU_CFG_SPI_MST_SW_MODE_SW_SPI_CS;
        writel(sw_mode, VTSS_ICPU_CFG_SPI_MST_SW_MODE);
        
	msg->state = NULL;
	if (msg->complete)
		msg->complete(msg->context);
}

static void int_error_stop(struct jaguar2_spi *jaguar2s, const char *msg)
{
	/* Stop the hw */
	spi_enable_chip(jaguar2s, 0);

	dev_err(&jaguar2s->master->dev, "%s\n", msg);
	jaguar2s->cur_msg->state = ERROR_STATE;
	tasklet_schedule(&jaguar2s->pump_transfers);
}

void jaguar2_spi_xfer_done(struct jaguar2_spi *jaguar2s)
{
	/* Update total byte transferred return count actual bytes read */
	jaguar2s->cur_msg->actual_length += jaguar2s->len;

	/* Move to next transfer */
	jaguar2s->cur_msg->state = next_transfer(jaguar2s);

	/* Handle end of message */
	if (jaguar2s->cur_msg->state == DONE_STATE) {
		jaguar2s->cur_msg->status = 0;
		giveback(jaguar2s);
	} else
		tasklet_schedule(&jaguar2s->pump_transfers);
}
EXPORT_SYMBOL_GPL(jaguar2_spi_xfer_done);

static irqreturn_t interrupt_transfer(struct jaguar2_spi *jaguar2s)
{
	u32 irq_status = readl(VTSS_SIMC_SIMC_ISR);

	/* Error handling */
	if (irq_status & (SPI_INT_TXOI | SPI_INT_RXOI | SPI_INT_RXUI)) {
		readl(VTSS_SIMC_SIMC_TXOICR);
		readl(VTSS_SIMC_SIMC_RXOICR);
		readl(VTSS_SIMC_SIMC_RXUICR);
		int_error_stop(jaguar2s, "interrupt_transfer: fifo overrun/underrun");
		return IRQ_HANDLED;
	}

	jaguar2_reader(jaguar2s);
	if (jaguar2s->rx_end == jaguar2s->rx) {
		spi_mask_intr(jaguar2s, SPI_INT_TXEI);
		jaguar2_spi_xfer_done(jaguar2s);
		return IRQ_HANDLED;
	}
	if (irq_status & SPI_INT_TXEI) {
		spi_mask_intr(jaguar2s, SPI_INT_TXEI);
		jaguar2_writer(jaguar2s);
		/* Enable TX irq always, it will be disabled when RX finished */
		spi_umask_intr(jaguar2s, SPI_INT_TXEI);
	}

	return IRQ_HANDLED;
}

static irqreturn_t jaguar2_spi_irq(int irq, void *dev_id)
{
	struct jaguar2_spi *jaguar2s = dev_id;
	u32 irq_status = readl(VTSS_SIMC_SIMC_ISR) & 0x3f;

	if (!irq_status)
		return IRQ_NONE;

	if (!jaguar2s->cur_msg) {
		spi_mask_intr(jaguar2s, SPI_INT_TXEI);
		return IRQ_HANDLED;
	}

	return jaguar2s->transfer_handler(jaguar2s);
}

/* Must be called inside pump_transfers() */
static void poll_transfer(struct jaguar2_spi *jaguar2s)
{
	do {
		jaguar2_writer(jaguar2s);
		jaguar2_reader(jaguar2s);
		cpu_relax();
	} while (jaguar2s->rx_end > jaguar2s->rx);

	jaguar2_spi_xfer_done(jaguar2s);
}

static void pump_transfers(unsigned long data)
{
	struct jaguar2_spi *jaguar2s = (struct jaguar2_spi *)data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct spi_device *spi = NULL;
	struct chip_data *chip = NULL;
	u8 bits = 0;
	u8 imask = 0;
	u8 cs_change = 0;
	u32 txint_level = 0;
	u32 clk_div = 250;
	u32 speed = 1000000;  // 1 MHz
	u32 cr0 = 0;

	/* Get current state information */
	message = jaguar2s->cur_msg;
	transfer = jaguar2s->cur_transfer;
	chip = jaguar2s->cur_chip;
	spi = message->spi;

	if (unlikely(!chip->clk_div))
		chip->clk_div = jaguar2s->max_freq / chip->speed_hz;

	if (message->state == ERROR_STATE) {
		message->status = -EIO;
		goto early_exit;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		message->status = 0;
		goto early_exit;
	}

	/* Delay if requested at end of transfer*/
	if (message->state == RUNNING_STATE) {
		previous = list_entry(transfer->transfer_list.prev,
					struct spi_transfer,
					transfer_list);
		if (previous->delay_usecs)
			udelay(previous->delay_usecs);
	}

	jaguar2s->n_bytes = chip->n_bytes;
	jaguar2s->cs_control = chip->cs_control;
	jaguar2s->tx = (void *)transfer->tx_buf;
	jaguar2s->tx_end = jaguar2s->tx + transfer->len;
	jaguar2s->rx = transfer->rx_buf;
	jaguar2s->rx_end = jaguar2s->rx + transfer->len;
	jaguar2s->len = jaguar2s->cur_transfer->len;
	if (chip != jaguar2s->prev_chip)
		cs_change = 1;

	cr0 = chip->cr0;

	/* Handle per transfer options for bpw and speed */
	if (transfer->speed_hz) {
		speed = chip->speed_hz;

		if (transfer->speed_hz != speed) {
			speed = transfer->speed_hz;
			if (speed > jaguar2s->max_freq) {
				printk(KERN_ERR "MRST SPI0: unsupported"
					"freq: %dHz\n", speed);
				message->status = -EIO;
				goto early_exit;
			}

			/* clk_div doesn't support odd number */
			clk_div = jaguar2s->max_freq / speed;
			clk_div = (clk_div + 1) & 0xfffe;

			chip->speed_hz = speed;
			chip->clk_div = clk_div;
		}
	}
	if (transfer->bits_per_word) {
		bits = transfer->bits_per_word;
		jaguar2s->n_bytes = bits >> 3;
		cr0 = (bits - 1)
			| (chip->type << SPI_FRF_OFFSET)
			| (spi->mode << SPI_MODE_OFFSET)
			| (chip->tmode << SPI_TMOD_OFFSET);
	}
	message->state = RUNNING_STATE;

	/*
	 * Adjust transfer mode if necessary. Requires platform dependent
	 * chipselect mechanism.
	 */
	if (jaguar2s->cs_control) {
		if (jaguar2s->rx && jaguar2s->tx)
			chip->tmode = SPI_TMOD_TR;
		else if (jaguar2s->rx)
			chip->tmode = SPI_TMOD_RO;
		else
			chip->tmode = SPI_TMOD_TO;

		cr0 &= ~SPI_TMOD_MASK;
		cr0 |= (chip->tmode << SPI_TMOD_OFFSET);
	}

	/*
	 * Interrupt mode
	 * we only need set the TXEI IRQ, as TX/RX always happen syncronizely
	 */
	if (!chip->poll_mode) {
		int templen = jaguar2s->len / jaguar2s->n_bytes;
		txint_level = jaguar2s->fifo_len / 2;
		txint_level = (templen > txint_level) ? txint_level : templen;

		imask |= SPI_INT_TXEI | SPI_INT_TXOI | SPI_INT_RXUI | SPI_INT_RXOI;
		jaguar2s->transfer_handler = interrupt_transfer;
	}

	/*
	 * Reprogram registers only if
	 *	1. chip select changes
	 *	2. clk_div is changed
	 *	3. control value changes
	 */
	if (readl(VTSS_SIMC_SIMC_CTRLR0) != cr0 || cs_change || clk_div || imask) {
		spi_enable_chip(jaguar2s, 0);

		if (readl(VTSS_SIMC_SIMC_CTRLR0) != cr0)
                    writel(cr0, VTSS_SIMC_SIMC_CTRLR0);

		spi_set_clk(jaguar2s, clk_div ? clk_div : chip->clk_div);
                
		spi_chip_sel(jaguar2s, spi->chip_select);

		/* Set the interrupt mask, for poll mode just disable all int */
		spi_mask_intr(jaguar2s, 0xff);
		if (imask)
			spi_umask_intr(jaguar2s, imask);
		if (txint_level)
                    writel(txint_level, VTSS_SIMC_SIMC_TXFTLR);

		spi_enable_chip(jaguar2s, 1);
		if (cs_change)
			jaguar2s->prev_chip = chip;
	}

	if (chip->poll_mode)
                poll_transfer(jaguar2s);

	return;

early_exit:
	giveback(jaguar2s);
	return;
}

static void pump_messages(struct work_struct *work)
{
	struct jaguar2_spi *jaguar2s =
		container_of(work, struct jaguar2_spi, pump_messages);
	unsigned long flags;

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&jaguar2s->lock, flags);
	if (list_empty(&jaguar2s->queue) || jaguar2s->run == QUEUE_STOPPED) {
		jaguar2s->busy = 0;
		spin_unlock_irqrestore(&jaguar2s->lock, flags);
		return;
	}

	/* Make sure we are not already running a message */
	if (jaguar2s->cur_msg) {
		spin_unlock_irqrestore(&jaguar2s->lock, flags);
		return;
	}

	/* Extract head of queue */
	jaguar2s->cur_msg = list_entry(jaguar2s->queue.next, struct spi_message, queue);
	list_del_init(&jaguar2s->cur_msg->queue);

	/* Initial message state*/
	jaguar2s->cur_msg->state = START_STATE;
	jaguar2s->cur_transfer = list_entry(jaguar2s->cur_msg->transfers.next,
						struct spi_transfer,
						transfer_list);
	jaguar2s->cur_chip = spi_get_ctldata(jaguar2s->cur_msg->spi);

	/* Mark as busy and launch transfers */
	tasklet_schedule(&jaguar2s->pump_transfers);

	jaguar2s->busy = 1;
	spin_unlock_irqrestore(&jaguar2s->lock, flags);
}

/* spi_device use this to queue in their spi_msg */
static int jaguar2_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct jaguar2_spi *jaguar2s = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&jaguar2s->lock, flags);

	if (jaguar2s->run == QUEUE_STOPPED) {
		spin_unlock_irqrestore(&jaguar2s->lock, flags);
		return -ESHUTDOWN;
	}

	msg->actual_length = 0;
	msg->status = -EINPROGRESS;
	msg->state = START_STATE;

	list_add_tail(&msg->queue, &jaguar2s->queue);

	if (jaguar2s->run == QUEUE_RUNNING && !jaguar2s->busy) {

		if (jaguar2s->cur_transfer || jaguar2s->cur_msg)
			queue_work(jaguar2s->workqueue,
					&jaguar2s->pump_messages);
		else {
			/* If no other data transaction in air, just go */
			spin_unlock_irqrestore(&jaguar2s->lock, flags);
			pump_messages(&jaguar2s->pump_messages);
			return 0;
		}
	}

	spin_unlock_irqrestore(&jaguar2s->lock, flags);
	return 0;
}

/* This may be called twice for each spi dev */
static int jaguar2_spi_setup(struct spi_device *spi)
{
	struct jaguar2_spi_chip *chip_info = NULL;
	struct chip_data *chip;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
                chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi_set_ctldata(spi, chip);
	}

	/*
	 * Protocol drivers may change the chip settings, so...
	 * if chip_info exists, use it
	 */
	chip_info = spi->controller_data;

	/* chip_info doesn't always exist */
	if (chip_info) {
		if (chip_info->cs_control)
			chip->cs_control = chip_info->cs_control;

		chip->poll_mode = chip_info->poll_mode;
		chip->type = chip_info->type;

		chip->rx_threshold = 0;
		chip->tx_threshold = 0;
	}

	if (spi->bits_per_word == 8) {
		chip->n_bytes = 1;
	} else if (spi->bits_per_word == 16) {
		chip->n_bytes = 2;
	}
	chip->bits_per_word = spi->bits_per_word;

	if (!spi->max_speed_hz) {
		dev_err(&spi->dev, "No max speed HZ parameter\n");
		return -EINVAL;
	}
	chip->speed_hz = spi->max_speed_hz;

	chip->tmode = 0; /* Tx & Rx */
	/* Default SPI mode is SCPOL = 0, SCPH = 0 */
	chip->cr0 = (chip->bits_per_word - 1)
			| (chip->type << SPI_FRF_OFFSET)
			| (spi->mode  << SPI_MODE_OFFSET)
			| (chip->tmode << SPI_TMOD_OFFSET);

	return 0;
}

static void jaguar2_spi_cleanup(struct spi_device *spi)
{
        /* cleanup for kalloc */
        struct chip_data *chip = spi_get_ctldata(spi);

        kfree(chip);
        spi_set_ctldata(spi, NULL);
}

static int init_queue(struct jaguar2_spi *jaguar2s)
{
	INIT_LIST_HEAD(&jaguar2s->queue);
	spin_lock_init(&jaguar2s->lock);

	jaguar2s->run = QUEUE_STOPPED;
	jaguar2s->busy = 0;

	tasklet_init(&jaguar2s->pump_transfers,
			pump_transfers,	(unsigned long)jaguar2s);

        
	INIT_WORK(&jaguar2s->pump_messages, pump_messages);
	jaguar2s->workqueue = create_singlethread_workqueue(
					dev_name(jaguar2s->master->dev.parent));
	if (jaguar2s->workqueue == NULL)
		return -EBUSY;
        
	return 0;
}

static int start_queue(struct jaguar2_spi *jaguar2s)
{
	unsigned long flags;

	spin_lock_irqsave(&jaguar2s->lock, flags);

	if (jaguar2s->run == QUEUE_RUNNING || jaguar2s->busy) {
                spin_unlock_irqrestore(&jaguar2s->lock, flags);
		return -EBUSY;
	}

	jaguar2s->run = QUEUE_RUNNING;
	jaguar2s->cur_msg = NULL;
	jaguar2s->cur_transfer = NULL;
	jaguar2s->cur_chip = NULL;
	jaguar2s->prev_chip = NULL;

	spin_unlock_irqrestore(&jaguar2s->lock, flags);
        
	queue_work(jaguar2s->workqueue, &jaguar2s->pump_messages);
        
	return 0;
}

static int stop_queue(struct jaguar2_spi *jaguar2s)
{
	unsigned long flags;
	unsigned limit = 50;
	int status = 0;

	spin_lock_irqsave(&jaguar2s->lock, flags);
	jaguar2s->run = QUEUE_STOPPED;


	while ((!list_empty(&jaguar2s->queue) || jaguar2s->busy) && limit--) {
		spin_unlock_irqrestore(&jaguar2s->lock, flags);
		msleep(10);
		spin_lock_irqsave(&jaguar2s->lock, flags);
	}
        
	if (!list_empty(&jaguar2s->queue) || jaguar2s->busy)
		status = -EBUSY;
	spin_unlock_irqrestore(&jaguar2s->lock, flags);

	return status;
}

static int destroy_queue(struct jaguar2_spi *jaguar2s)
{
	int status;

	status = stop_queue(jaguar2s);
	if (status != 0)
		return status;
	destroy_workqueue(jaguar2s->workqueue);
	return 0;
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void spi_hw_init(struct jaguar2_spi *jaguar2s)
{
        spi_enable_chip(jaguar2s, 0);
        /* mask all the interrupts  */
	spi_mask_intr(jaguar2s, 0xff);
	spi_enable_chip(jaguar2s, 1);

        /* The transmit FIFO depth is 8, do not program value exceeding 7.
         * By default, fifo_len is set to 7.
         */
        spi_set_fifo_len(jaguar2s, 7);        
        
	/*
	 * Try to detect the FIFO depth if not set by interface driver,
	 * the depth could be from 2 to 256 from HW spec
	 */
	if (!jaguar2s->fifo_len) {
		u32 fifo;
		for (fifo = 2; fifo <= 257; fifo++) {
                    writel(fifo, VTSS_SIMC_SIMC_TXFTLR);
			if (fifo != readl(VTSS_SIMC_SIMC_TXFTLR))
				break;
		}

		jaguar2s->fifo_len = (fifo == 257) ? 0 : fifo;
		writel(0, VTSS_SIMC_SIMC_TXFTLR);
	}
}

int jaguar2_spi_add_host(struct device *dev, struct jaguar2_spi *jaguar2s)
{
	struct spi_master *master;
	int ret;

	BUG_ON(jaguar2s == NULL);

	master = spi_alloc_master(dev, 0);
	if (!master)
            return -ENOMEM;

	jaguar2s->master = master;
	jaguar2s->type = SSI_MOTO_SPI;
        jaguar2s->max_freq = 25000000;  // the maximum baud rate is 25 MHz
	jaguar2s->prev_chip = NULL;
	snprintf(jaguar2s->name, sizeof(jaguar2s->name), "jaguar2_spi%d",
			jaguar2s->bus_num);

	ret = devm_request_irq(dev, jaguar2s->irq, jaguar2_spi_irq, IRQF_SHARED,
			jaguar2s->name, jaguar2s);
	if (ret < 0) {
		dev_err(&master->dev, "can not get IRQ\n");
		goto err_free_master;
	}

	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);
	master->bus_num = jaguar2s->bus_num;
	master->num_chipselect = jaguar2s->num_cs;
	master->setup = jaguar2_spi_setup;
        master->cleanup = jaguar2_spi_cleanup;
	master->transfer = jaguar2_spi_transfer;

        /* Make SI master controller the owner of SI interface */
        spi_set_owner(jaguar2s, 2);
        
	/* Basic HW init */
	spi_hw_init(jaguar2s);

	/* Initial and start queue */
	ret = init_queue(jaguar2s);
	if (ret) {
		dev_err(&master->dev, "problem initializing queue\n");
		goto err_diable_hw;
	}

	ret = start_queue(jaguar2s);
	if (ret) {
		dev_err(&master->dev, "problem starting queue\n");
		goto err_diable_hw;
	}

	spi_master_set_devdata(master, jaguar2s);
	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_queue_alloc;
	}        

	mrst_spi_debugfs_init(jaguar2s);
	return 0;

err_queue_alloc:
	destroy_queue(jaguar2s);
err_diable_hw:
	spi_enable_chip(jaguar2s, 0);
err_free_master:
	spi_master_put(master);
	return ret;
}
EXPORT_SYMBOL_GPL(jaguar2_spi_add_host);

void jaguar2_spi_remove_host(struct jaguar2_spi *jaguar2s)
{
	int status = 0;

	if (!jaguar2s)
		return;
	mrst_spi_debugfs_remove(jaguar2s);

	/* Remove the queue */
	status = destroy_queue(jaguar2s);
	if (status != 0)
		dev_err(&jaguar2s->master->dev,
			"jaguar2_spi_remove: workqueue will not complete, message memory not freed\n");

	spi_enable_chip(jaguar2s, 0);
	/* Disable clk */
	spi_set_clk(jaguar2s, 0);
}
EXPORT_SYMBOL_GPL(jaguar2_spi_remove_host);

int jaguar2_spi_suspend_host(struct jaguar2_spi *jaguar2s)
{
	int ret = 0;

	ret = stop_queue(jaguar2s);
	if (ret)
		return ret;
	spi_enable_chip(jaguar2s, 0);
	spi_set_clk(jaguar2s, 0);
	return ret;
}
EXPORT_SYMBOL_GPL(jaguar2_spi_suspend_host);

int jaguar2_spi_resume_host(struct jaguar2_spi *jaguar2s)
{
	int ret;

	spi_hw_init(jaguar2s);
	ret = start_queue(jaguar2s);
	if (ret)
		dev_err(&jaguar2s->master->dev, "fail to start queue (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(jaguar2_spi_resume_host);

MODULE_AUTHOR("Wenxi Jin <wjin@vitesse.com>");
MODULE_DESCRIPTION("Driver for SPI master controller on Jaguar2 platform");
MODULE_LICENSE("GPL v2");
