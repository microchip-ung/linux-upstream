#ifndef SPI_JAGUAR2_HEADER_H
#define SPI_JAGUAR2_HEADER_H

#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/spi/spi_jaguar2.h>
#include <asm/mach-jaguar2/hardware.h>

/* Bit fields in CTRLR0 */
#define SPI_FRF_OFFSET			4
#define SPI_FRF_SPI			0x0
#define SPI_FRF_SSP			0x1
#define SPI_FRF_RESV1                   0x2
#define SPI_FRF_RESV2			0x3

#define SPI_MODE_OFFSET			6
#define SPI_SCPH_OFFSET			6
#define SPI_SCOL_OFFSET			7

#define SPI_TMOD_OFFSET			8
#define SPI_TMOD_MASK			(0x3 << SPI_TMOD_OFFSET)
#define	SPI_TMOD_TR			0x0		/* xmit & recv */
#define SPI_TMOD_TO			0x1		/* xmit only */
#define SPI_TMOD_RO			0x2		/* recv only */
#define SPI_TMOD_EPROMREAD		0x3		/* eeprom read mode */

#define SPI_SLVOE_OFFSET		10
#define SPI_SRL_OFFSET			11
#define SPI_CFS_OFFSET			12

/* Bit fields in SR, 7 bits */
#define SR_MASK				0x7f		/* cover 7 bits */
#define SR_BUSY				(1 << 0)
#define SR_TF_NOT_FULL			(1 << 1)
#define SR_TF_EMPT			(1 << 2)
#define SR_RF_NOT_EMPT			(1 << 3)
#define SR_RF_FULL			(1 << 4)
#define SR_TX_ERR			(1 << 5)
#define SR_DCOL				(1 << 6)

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI			(1 << 0)
#define SPI_INT_TXOI			(1 << 1)
#define SPI_INT_RXUI			(1 << 2)
#define SPI_INT_RXOI			(1 << 3)
#define SPI_INT_RXFI			(1 << 4)
#define SPI_INT_MSTI			(1 << 5)

/* TX RX interrupt level threshold, max can be 256 */
#define SPI_INT_THRESHOLD		32

enum jaguar2_ssi_type {
	SSI_MOTO_SPI = 0,
	SSI_TI_SSP,
	SSI_NS_MICROWIRE,
};

struct jaguar2_spi {
	struct spi_master	*master;
	struct spi_device	*cur_dev;
	enum jaguar2_ssi_type	type;
	char			name[16];

	void __iomem		*regs;
	unsigned long		paddr;
	int			irq;
	u32			fifo_len;	/* depth of the FIFO buffer */
	u32			max_freq;	/* max bus freq supported */

	u16			bus_num;
	u16			num_cs;		/* supported slave numbers */

	/* Driver message queue */
	struct workqueue_struct	*workqueue;
	struct work_struct	pump_messages;
	spinlock_t		lock;
	struct list_head	queue;
	int			busy;
	int			run;

	/* Message Transfer pump */
	struct tasklet_struct	pump_transfers;

	/* Current message transfer state info */
	struct spi_message	*cur_msg;
	struct spi_transfer	*cur_transfer;
	struct chip_data	*cur_chip;
	struct chip_data	*prev_chip;
	size_t			len;
	void			*tx;
	void			*tx_end;
	void			*rx;
	void			*rx_end;
	size_t			rx_map_len;
	size_t			tx_map_len;
	u8			n_bytes;	/* current is a 1/2 bytes op */
	u8			max_bits_per_word;	/* maxim is 16b */
	/* u32			dma_width; */
	irqreturn_t		(*transfer_handler)(struct jaguar2_spi *jaguar2s);
	void			(*cs_control)(u32 command);

	/* Bus interface info */
	void			*priv;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
#endif
        u32                     bb_cur;
};

static inline struct jaguar2_spi *spidev_to_sg(struct spi_master *master)
{
    return spi_master_get_devdata(master);
}

/* Make SI master controller the owner of the SI interface */
static inline void spi_set_owner(struct jaguar2_spi *jaguar2s, u32 val)
{
    u32 general_ctrl = readl(VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL);

    /* IF_SI0_OWNER, select the owner of the SI interface
     * Encoding: 0: SI Slave
     *           1: SI Boot Master
     *           2: SI Master Controller
     */
    general_ctrl |= VTSS_F_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL_IF_SI_OWNER(val);
    writel(general_ctrl, VTSS_ICPU_CFG_CPU_SYSTEM_CTRL_GENERAL_CTRL);
}

/* Set 16-bit data frame */
static inline void spi_set_dataframe(struct jaguar2_spi *jaguar2s, u32 bits)
{
    struct jaguar2_spi *sp = spidev_to_sg(jaguar2s->master);

    /* Selects the data frame length. See SIMC::DR register description for how to read/write words of less than 16 bit.
     * Encoding: 0-2: Reserved
     *           n: n+1 bit serial data transfer
     * Default:  0x7
     */
    sp->bb_cur |= VTSS_F_SIMC_SIMC_CTRLR0_DFS(bits);
    writel(sp->bb_cur, VTSS_SIMC_SIMC_CTRLR0);
}

/* Set mode for serial transmision */
static inline void spi_set_tmod(struct jaguar2_spi *jaguar2s, u32 tmod)
{
    struct jaguar2_spi *sp = spidev_to_sg(jaguar2s->master);
    /*
     * Select the mode of transfer for serial communication.
     * Encoding: 0: Transmit and Receive.
     *           1: Transmit only
     *           2: Reveive only
     *           3: Reserved
     * Default:  0x0
     */
    sp->bb_cur |= VTSS_F_SIMC_SIMC_CTRLR0_TMOD(tmod);
    writew(sp->bb_cur, VTSS_SIMC_SIMC_CTRLR0);
}

static inline void spi_enable_chip(struct jaguar2_spi *jaguar2s, u32 enable)
{
    struct jaguar2_spi *sp = spidev_to_sg(jaguar2s->master);
            
    if (enable) {
        sp->bb_cur |= VTSS_F_SIMC_SIMC_SIMCEN_SIMCEN(1);
    }
    else
        sp->bb_cur &= ~VTSS_M_SIMC_SIMC_SIMCEN_SIMCEN;

    writew(sp->bb_cur, VTSS_SIMC_SIMC_SIMCEN);
}

static inline void spi_set_fifo_len(struct jaguar2_spi *jaguar2s, u32 fifo_len)
{
    struct jaguar2_spi *sp = spidev_to_sg(jaguar2s->master);

    if (fifo_len < 0) {
        printk(KERN_ERR "invalid fifo depth!\n");
    } else if (fifo_len > 7) {
        jaguar2s->fifo_len = 7; // fifo depth must be less than 8
    } else {
        jaguar2s->fifo_len = fifo_len;
    }

    sp->bb_cur |= VTSS_F_SIMC_SIMC_TXFTLR_TFT(jaguar2s->fifo_len);
    writew(sp->bb_cur, VTSS_SIMC_SIMC_TXFTLR);
}

static inline void spi_set_clk(struct jaguar2_spi *jaguar2s, u32 div)
{
    struct jaguar2_spi *sp = spidev_to_sg(jaguar2s->master);
    sp->bb_cur = VTSS_F_SIMC_SIMC_BAUDR_SCKDV(div);
    writew(sp->bb_cur, VTSS_SIMC_SIMC_BAUDR);
}

static inline void spi_chip_sel(struct jaguar2_spi *jaguar2s, u32 cs)
{
    if (cs > jaguar2s->num_cs)
        return;

    if (jaguar2s->cs_control)
        jaguar2s->cs_control(1);
    writew(1 << cs, VTSS_SIMC_SIMC_SER);
}

/* Disable IRQ bits */
static inline void spi_mask_intr(struct jaguar2_spi *jaguar2s, u32 mask)
{
	u32 new_mask;

	new_mask = readl(VTSS_SIMC_SIMC_IMR) & ~mask;
	writel(new_mask, VTSS_SIMC_SIMC_IMR);
}

/* Enable IRQ bits */
static inline void spi_umask_intr(struct jaguar2_spi *jaguar2s, u32 mask)
{
	u32 new_mask;

	new_mask = readl(VTSS_SIMC_SIMC_IMR) | mask;
	writel(new_mask, VTSS_SIMC_SIMC_IMR);
}

/*
 * Each SPI slave device to work with jaguar2_spi controller should
 * has such a structure claiming its working mode (PIO/DMA etc),
 * which can be save in the "controller_data" member of the
 * struct spi_device
 */
struct jaguar2_spi_chip {
	u8 poll_mode;	/* 0 for contoller polling mode */
	u8 type;	/* SPI/SSP/Micrwire */
	void (*cs_control)(u32 command);
};

extern int jaguar2_spi_add_host(struct device *dev, struct jaguar2_spi *jaguar2s);
extern void jaguar2_spi_remove_host(struct jaguar2_spi *jaguar2s);
extern int jaguar2_spi_suspend_host(struct jaguar2_spi *jaguar2s);
extern int jaguar2_spi_resume_host(struct jaguar2_spi *jaguar2s);
extern void jaguar2_spi_xfer_done(struct jaguar2_spi *jaguar2s);

/* platform related setup */
extern int jaguar2_spi_mid_init(struct jaguar2_spi *jaguar2s); /* Intel MID platforms */
#endif /* SPI_JAGUAR2_HEADER_H */
