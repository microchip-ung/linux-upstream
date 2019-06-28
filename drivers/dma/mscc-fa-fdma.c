// SPDX-License-Identifier: (GPL-2.0 OR MIT)
//
// Microsemi SoC Fireant FDMA driver
//
// Copyright (c) 2019 Microsemi

#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/pci.h>

#include "dmaengine.h"

#define DEVICE_NAME "fireant-fdma"

/* Register offsets */

enum fireant_top {
    FIREANT_DEFAULT   = 0x000000000, /* Default instance in CML model */
    FIREANT_AMBA_TOP  = 0x010000000, /* amba_top instance in CML model */
};

#define FIREANT_REGIONS                            2
#define GET_REGION(off)                            ((off) & GENMASK(31, 28))
#define GET_REGION_INDEX(off)                      ((off) >> 28)
#define GET_ADDRESS(off)                           ((off) & GENMASK(27, 0))

#define AMBA_TOP_SPACE                             0x600000000
#define VCORE_CSR_SPACE                            0x610000000
#define FDMA_ATU_TARGET_SPACE                      0x900000000


/* ASM:CFG:PORT_CFG[67] */
#define ASM_CFG_PORT_R_OFF(ridx)                   (0x0060841c + (ridx*4))

/* CPU:CPU_REGS:GPR[32] */
#define CPU_REGS_GPR_R_OFF(ridx)                   (0x10000000 + (ridx*4))

/* DEVCPU_GCB:CHIP_REGS:CHIP_ID */
#define DEVCPU_GCB_CHIP_REGS_ID_OFF                0x01010000
/* DEVCPU_GCB:VCORE_ACCESS:VA_CTRL */
#define DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_OFF        0x010101c8
/* DEVCPU_GCB:VCORE_ACCESS:VA_ADDR_LSB */
#define DEVCPU_GCB_VCORE_ACCESS_VA_ADDR_LSB_OFF    0x010101cc
/* DEVCPU_GCB:VCORE_ACCESS:VA_ADDR_MSB */
#define DEVCPU_GCB_VCORE_ACCESS_VA_ADDR_MSB_OFF    0x010101d0
/* DEVCPU_GCB:VCORE_ACCESS:VA_DATA */
#define DEVCPU_GCB_VCORE_ACCESS_VA_DATA_OFF        0x010101d4
/* DEVCPU_GCB:VCORE_ACCESS:VA_DATA_INERT */
#define DEVCPU_GCB_VCORE_ACCESS_VA_DATA_INERT_OFF  0x010101dc

/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_R_OFF(ridx)          (0x01030000 + (ridx*4))
/* DEVCPU_QS:INJ:INJ_GRP_CFG[2] */
#define DEVCPU_QS_INJ_GRP_CFG_R_OFF(ridx)          (0x01030024 + (ridx*4))
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_R_OFF(ridx)             (0x01030034 + (ridx*4))

/* FDMA:FDMA:FDMA_CH_ACTIVATE */
#define FDMA_CH_ACTIVATE_OFF                       0x10080008
/* FDMA:FDMA:FDMA_CH_RELOAD */
#define FDMA_CH_RELOAD_OFF                         0x1008000c
/* FDMA:FDMA:FDMA_CH_DISABLE */
#define FDMA_CH_DISABLE_OFF                        0x10080010
/* FDMA:FDMA:FDMA_CH_FORCEDIS */
#define FDMA_CH_FORCEDIS_OFF                       0x10080014
/* FDMA:FDMA:FDMA_CH_DB_DISCARD */
#define FDMA_CH_DB_DISCARD_OFF                     0x10080018
/* FDMA:FDMA:FDMA_CH_CNT[8] */
#define FDMA_CH_CNT_R_OFF(ridx)                    (0x1008001c + (ridx*4))
/* FDMA:FDMA:FDMA_DCB_LLP[8] */
#define FDMA_DCB_LLP_R_OFF(ridx)                   (0x1008003c + (ridx*4))
/* FDMA:FDMA:FDMA_DCB_LLP1[8] */
#define FDMA_DCB_LLP1_R_OFF(ridx)                  (0x1008005c + (ridx*4))
/* FDMA:FDMA:FDMA_DCB_LLP_PREV[8] */
#define FDMA_DCB_LLP_PREV_R_OFF(ridx)              (0x1008007c + (ridx*4))
/* FDMA:FDMA:FDMA_DCB_LLP_PREV1[8] */
#define FDMA_DCB_LLP_PREV1_R_OFF(ridx)             (0x1008009c + (ridx*4))
/* FDMA:FDMA:FDMA_CH_ACTIVE */
#define FDMA_CH_ACTIVE_OFF                         0x100800bc
/* FDMA:FDMA:FDMA_CH_PENDING */
#define FDMA_CH_PENDING_OFF                        0x100800c0
/* FDMA:FDMA:FDMA_CH_IDLE */
#define FDMA_CH_IDLE_OFF                           0x100800c4
/* FDMA:FDMA:FDMA_CH_STATUS[8] */
#define FDMA_CH_STATUS_R_OFF(ridx)                 (0x100800c8 + (ridx*4))
/* FDMA:FDMA:FDMA_CH_CFG[8] */
#define FDMA_CH_CFG_R_OFF(ridx)                    (0x100800e8 + (ridx*4))
/* FDMA:FDMA:FDMA_CH_TRANSLATE[8] */
#define FDMA_CH_TRANSLATE_R_OFF(ridx)              (0x10080108 + (ridx*4))
/* FDMA:FDMA:FDMA_CH_INJ_TOKEN_CNT[6] */
#define FDMA_CH_INJ_TOKEN_CNT_R_OFF(ridx)          (0x10080128 + (ridx*4))
/* FDMA:FDMA:FDMA_CH_INJ_TOKEN_TICK_RLD[6] */
#define FDMA_CH_INJ_TOKEN_TICK_RLD_R_OFF(ridx)     (0x10080140 + (ridx*4))
/* FDMA:FDMA:FDMA_CH_INJ_TOKEN_TICK_CNT[6] */
#define FDMA_CH_INJ_TOKEN_TICK_CNT_R_OFF(ridx)     (0x10080158 + (ridx*4))
/* FDMA:FDMA:FDMA_INJ_CFG */
#define FDMA_INJ_CFG_OFF                           0x10080170
/* FDMA:FDMA:FDMA_XTR_CFG */
#define FDMA_XTR_CFG_OFF                           0x10080174
/* FDMA:FDMA:FDMA_PORT_CFG[2] */
#define FDMA_PORT_CFG_R_OFF(ridx)                  (0x10080178 + (ridx*4))
/* FDMA:FDMA:FDMA_PORT_CTRL[2] */
#define FDMA_PORT_CTRL_R_OFF(ridx)                 (0x10080180 + (ridx*4))
/* FDMA:FDMA:FDMA_INTR_DCB */
#define FDMA_INTR_DCB_OFF                          0x10080188
/* FDMA:FDMA:FDMA_INTR_DCB_ENA */
#define FDMA_INTR_DCB_ENA_OFF                      0x1008018c
/* FDMA:FDMA:FDMA_INTR_DB */
#define FDMA_INTR_DB_OFF                           0x10080190
/* FDMA:FDMA:FDMA_INTR_DB_ENA */
#define FDMA_INTR_DB_ENA_OFF                       0x10080194
/* FDMA:FDMA:FDMA_INTR_ERR */
#define FDMA_INTR_ERR_OFF                          0x10080198
/* FDMA:FDMA:FDMA_INTR_ENA */
#define FDMA_INTR_ENA_OFF                          0x1008019c
/* FDMA:FDMA:FDMA_INTR_IDENT */
#define FDMA_INTR_IDENT_OFF                        0x100801a0
/* FDMA:FDMA:FDMA_ERRORS */
#define FDMA_ERRORS_OFF                            0x100801a4
/* FDMA:FDMA:FDMA_ERRORS_2 */
#define FDMA_ERRORS_2_OFF                          0x100801a8
/* FDMA:FDMA:FDMA_IDLECNT */
#define FDMA_IDLECNT_OFF                           0x100801ac
/* FDMA:FDMA:FDMA_CTRL */
#define FDMA_CTRL_OFF                              0x100801b0

/* PCIE_DM_EP:PF0_ATU_CAP:IATU_REGION_CTRL_2_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_REGION_CTRL_2_OFF_OUTBOUND_0_OFF          0x10700004
/* PCIE_DM_EP:PF0_ATU_CAP:IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_OFF          0x10700008
/* PCIE_DM_EP:PF0_ATU_CAP:IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0_OFF        0x1070000c
/* PCIE_DM_EP:PF0_ATU_CAP:IATU_LIMIT_ADDR_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_LIMIT_ADDR_OFF_OUTBOUND_0_OFF             0x10700010
/* PCIE_DM_EP:PF0_ATU_CAP:IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0_OFF        0x10700014
/* PCIE_DM_EP:PF0_ATU_CAP:IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0_OFF      0x10700018
/* PCIE_DM_EP:PF0_ATU_CAP:IATU_UPPR_LIMIT_ADDR_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_UPPR_LIMIT_ADDR_OFF_OUTBOUND_0_OFF        0x10700020

/* Accessors */


/* ASM:CFG:PORT_CFG[67] */
#define ASM_CFG_PORT_NO_PREAMBLE_ENA             BIT(9)
#define ASM_CFG_PORT_INJ_FORMAT(x)               (((x) << 2) & GENMASK(3, 2))
#define ASM_CFG_PORT_INJ_FORMAT_M                GENMASK(3, 2)
#define ASM_CFG_PORT_INJ_FORMAT_X(x)             (((x) & GENMASK(3, 2)) >> 2)


/* DEVCPU_GCB:VCORE_ACCESS:VA_CTRL */
#define DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_ERR_M    GENMASK(3, 2)
#define DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_ERR_X(x) (((x) & GENMASK(3, 2)) >> 2)
/* DEVCPU_GCB:VCORE_ACCESS:VA_CTRL */
#define DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_BUSY_RD  BIT(1)
/* DEVCPU_GCB:VCORE_ACCESS:VA_CTRL */
#define DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_BUSY     BIT(0)

/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_MODE(x)            (((x) << 2) & GENMASK(3, 2))
#define DEVCPU_QS_XTR_GRP_CFG_MODE_M             GENMASK(3, 2)
#define DEVCPU_QS_XTR_GRP_CFG_MODE_X(x)          (((x) & GENMASK(3, 2)) >> 2)
/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_STATUS_WORD_POS    BIT(1)
/* DEVCPU_QS:XTR:XTR_GRP_CFG[2] */
#define DEVCPU_QS_XTR_GRP_CFG_BYTE_SWAP          BIT(0)
/* DEVCPU_QS:INJ:INJ_GRP_CFG[2] */
#define DEVCPU_QS_INJ_GRP_CFG_MODE(x)            (((x) << 2) & GENMASK(3, 2))
#define DEVCPU_QS_INJ_GRP_CFG_MODE_M             GENMASK(3, 2)
#define DEVCPU_QS_INJ_GRP_CFG_MODE_X(x)          (((x) & GENMASK(3, 2)) >> 2)
/* DEVCPU_QS:INJ:INJ_GRP_CFG[2] */
#define DEVCPU_QS_INJ_GRP_CFG_BYTE_SWAP          BIT(0)
/* DEVCPU_QS:INJ:INJ_CTRL[2] */
#define DEVCPU_QS_INJ_CTRL_GAP_SIZE(x)           (((x) << 21) & GENMASK(24, 21))
#define DEVCPU_QS_INJ_CTRL_GAP_SIZE_M            GENMASK(24, 21)
#define DEVCPU_QS_INJ_CTRL_GAP_SIZE_X(x)         (((x) & GENMASK(24, 21)) >> 21)

/* FDMA:FDMA:FDMA_CH_CFG[8] */
#define FDMA_CH_CFG_INTR_DB_EOF_ONLY             BIT(6)
/* FDMA:FDMA:FDMA_CH_CFG[8] */
#define FDMA_CH_CFG_INJ_PORT                     BIT(5)
/* FDMA:FDMA:FDMA_CH_CFG[8] */
#define FDMA_CH_CFG_DCB_DB_CNT(x)                (((x) << 1) & GENMASK(4, 1))
#define FDMA_CH_CFG_DCB_DB_CNT_M                 GENMASK(4, 1)
#define FDMA_CH_CFG_DCB_DB_CNT_X(x)              (((x) & GENMASK(4, 1)) >> 1)
/* FDMA:FDMA:FDMA_CH_CFG[8] */
#define FDMA_CH_CFG_MEM                          BIT(0)
/* FDMA:FDMA:FDMA_PORT_CTRL[2] */
#define FDMA_PORT_CTRL_INJ_STOP                  BIT(4)
/* FDMA:FDMA:FDMA_PORT_CTRL[2] */
#define FDMA_PORT_CTRL_INJ_STOP_FORCE            BIT(3)
/* FDMA:FDMA:FDMA_PORT_CTRL[2] */
#define FDMA_PORT_CTRL_XTR_STOP                  BIT(2)
/* FDMA:FDMA:FDMA_PORT_CTRL[2] */
#define FDMA_PORT_CTRL_XTR_BUF_IS_EMPTY          BIT(1)
/* FDMA:FDMA:FDMA_PORT_CTRL[2] */
#define FDMA_PORT_CTRL_XTR_BUF_RST               BIT(0)
/* FDMA:FDMA:FDMA_CTRL */
#define FDMA_CTRL_NRESET                         BIT(0)


/* PCIE_DM_EP:PF0_ATU_CAP:IATU_REGION_CTRL_2_OFF_OUTBOUND_0 */
#define PCIE_DM_EP_PF0_ATU_REGION_CTRL_2_OFF_OUTBOUND_0_EN BIT(31)

/* DCB structure */

#define FDMA_DCB_MAX_DBS                         15
#define FDMA_DCB_INFO_DATAL(x)                   ((x) & GENMASK(15, 0))
#define FDMA_DCB_INFO_TOKEN                      BIT(17)
#define FDMA_DCB_INFO_INTR                       BIT(18)
#define FDMA_DCB_INFO_SW(x)                      (((x) << 24) & GENMASK(31, 24))

#define FDMA_DCB_STATUS_BLOCKL(x)                ((x) & GENMASK(15, 0))
#define FDMA_DCB_STATUS_SOF                      BIT(16)
#define FDMA_DCB_STATUS_EOF                      BIT(17)
#define FDMA_DCB_STATUS_INTR                     BIT(18)
#define FDMA_DCB_STATUS_DONE                     BIT(19)
#define FDMA_DCB_STATUS_BLOCKO(x)                (((x) << 20) & GENMASK(31, 20))
#define FDMA_DCB_INVALID_DATA                    0x1


/*
*                                        
*   +---------------------------+        
*   |         Next Ptr          |        
*   +---------------------------+        
*   |   Reserved  |    Info     |        
*   +---------------------------+        
*   |         Data0 Ptr         |        
*   +---------------------------+        
*   |   Reserved  |    Status0  |        
*   +---------------------------+        
*   |         Data1 Ptr         |        
*   +---------------------------+        
*   |   Reserved  |    Status1  |        
*   +---------------------------+        
*   |         Data2 Ptr         |        
*   +---------------------------+        
*   |   Reserved  |    Status2  |        
*   |-------------|-------------|        
*   |                           |        
*   |                           |        
*   |                           |        
*   |                           |        
*   |                           |        
*   |---------------------------|        
*   |         Data14 Ptr        |        
*   +-------------|-------------+        
*   |   Reserved  |    Status14 |        
*   +-------------|-------------+        
*/                                       

#define FDMA_BUFFER_ALIGN                        128
#define FDMA_BUFFER_MASK                         127
#define XTR_BUFFER_SIZE                          (XTR_CHUNK_SIZE*12)
#define DMA_RX_CHANNEL                           6
#define FDMA_DCB_MAX                             200
#define VCORE_ACCESS_TIMEOUT_MS                  5
#define FDMA_DISABLE_TIMEOUT_MS                  5

/* Debugging flags */
// #define FDMA_LOG
// #define FDMA_DCB_LOG
// #define FDMA_DCB_ERROR_LOG


struct mscc_fa_fdma;

enum mscc_fa_fdma_channel_state {
	DCS_IDLE = 0,
	DCS_ACTIVE,
	DCS_RUNNING,
	DCS_STOPPING,
	DCS_ERROR
};

enum mscc_fa_fdma_dcb_state {
	DCBS_IDLE = 0,    /* Not yet used or ready to be reused */
	DCBS_QUEUED,      /* In queue for transfer */
	DCBS_ERROR,       /* Transfer failed */
	DCBS_COMPLETE,    /* Transfer successful */
};

struct mscc_fa_fdma_data {
	u64 dataptr;
	u64 status;
};

struct mscc_fa_fdma_dcb_hw {
	u64 nextptr;
	u64 info;
	struct mscc_fa_fdma_data block[FDMA_DCB_MAX_DBS];
};

struct mscc_fa_fdma_block_info {
	int size;
};

struct mscc_fa_fdma_dcb {
	struct  mscc_fa_fdma_dcb_hw hw;
	struct dma_async_tx_descriptor txd;
	dma_addr_t phys;
	dma_addr_t offset;
	enum mscc_fa_fdma_dcb_state state;
	int valid_blocks;
	struct mscc_fa_fdma_dcb *first_dcb;
	int is_last_dcb;
	struct mscc_fa_fdma_block_info binfo[FDMA_DCB_MAX_DBS];
	struct list_head node;
};

struct mscc_fa_region {
	phys_addr_t phys_addr;
	void __iomem *io_addr;
	resource_size_t size;
	phys_addr_t vcore_addr;
};

struct mscc_fa_fdma_channel {
	struct dma_chan chan;
	enum mscc_fa_fdma_channel_state state;
	struct dma_tx_state tx_state;
	struct list_head queued_dcbs;
	int queue_state_dcbs;
	int completed_dcbs;
};

struct mscc_fa_fdma {
	struct dma_device dma;  /* Must be first member due to xlate function */
	struct mscc_fa_region regions[FIREANT_REGIONS];
	spinlock_t lock;
	struct dma_pool *dcb_pool;
	int irq;
	struct tasklet_struct tasklet;
	struct list_head free_dcbs;

	unsigned int nr_pchans;
	struct mscc_fa_fdma_channel chans[0];
};

static void mscc_fa_writel(struct mscc_fa_fdma *priv, u32 reg, u32 data)
{
	void __iomem *addr = 
		priv->regions[GET_REGION_INDEX(reg)].io_addr + GET_ADDRESS(reg);
#ifdef FDMA_LOG
	pr_debug("%s:%d %s: addr 0x%llx, data 0x%08x\n", 
		__FILE__, __LINE__, __func__,
		priv->regions[GET_REGION_INDEX(reg)].phys_addr + 
		GET_ADDRESS(reg),
		data);
#endif
	writel(data, addr);
}

static void mscc_fa_writell(struct mscc_fa_fdma *priv, u32 regls, u32 regms, 
			    u64 data)
{
	mscc_fa_writel(priv, regls, data & GENMASK(31,0));
	mscc_fa_writel(priv, regms, data >> 32);
}

static u32 mscc_fa_readl(struct mscc_fa_fdma *priv, u32 reg)
{
	void __iomem *addr = 
		priv->regions[GET_REGION_INDEX(reg)].io_addr + GET_ADDRESS(reg);
	u32 data = readl(addr);
#ifdef FDMA_LOG
	pr_debug("%s:%d %s: addr 0x%llx, data 0x%08x\n", 
		__FILE__, __LINE__, __func__, 
		priv->regions[GET_REGION_INDEX(reg)].phys_addr + 
		GET_ADDRESS(reg),
		data);
#endif
	return data;
}

#ifdef FDMA_DCB_ERROR_LOG
static u64 mscc_fa_readll(struct mscc_fa_fdma *priv, u32 regls, u32 regms)
{
	u32 datals = mscc_fa_readl(priv, regls);
	u64 datams = mscc_fa_readl(priv, regms);
	return (datams << 32) | datals;
}
#endif

static u32 mscc_fa_vcore_readl(struct mscc_fa_fdma *priv, u64 vcore_addr) {
	unsigned long deadline;
	u32 value;
	u32 status;

	/* Set VA_SIZE = 0 (32 bit access) */
	mscc_fa_writel(priv, DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_OFF, 0);
	/* Set LS Word */
	mscc_fa_writel(priv, DEVCPU_GCB_VCORE_ACCESS_VA_ADDR_LSB_OFF, 
		       (u32)vcore_addr);
	/* Set MS Word */
	mscc_fa_writel(priv, DEVCPU_GCB_VCORE_ACCESS_VA_ADDR_MSB_OFF, 
		       (u32)(vcore_addr >> 32));
	/* Read VA_DATA to trigger access */
	mscc_fa_readl(priv, DEVCPU_GCB_VCORE_ACCESS_VA_DATA_OFF);
	/* Wait while VA_BUSY is set, handle timeouts */
	deadline = jiffies + msecs_to_jiffies(VCORE_ACCESS_TIMEOUT_MS);
	do {
		status = mscc_fa_readl(priv, 
				       DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_OFF);
	} while (time_before(jiffies, deadline) &&
		 (status & DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_BUSY) &&
		 !(status & DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_ERR_M));
	if (status & DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_ERR_M) {
		pr_err("%s:%d %s: Error reading VCore Access at 0x%llx", 
			__FILE__, __LINE__, __func__, vcore_addr);
		return -EINVAL;
	}
	/* Read data without triggering a new read (using INERT) */
	value = mscc_fa_readl(priv, DEVCPU_GCB_VCORE_ACCESS_VA_DATA_INERT_OFF);
#ifdef FDMA_LOG
	pr_debug("%s:%d %s: VCore Access Read at 0x%llx, value 0x%08x", 
		__FILE__, __LINE__, __func__, vcore_addr, value);
#endif
	return value;
}

#ifdef FDMA_DCB_LOG

static u64 mscc_fa_vcore_readll(struct mscc_fa_fdma *priv, u64 vcore_addr)
{
	u32 datals = mscc_fa_vcore_readl(priv, vcore_addr);
	u64 datams = mscc_fa_vcore_readl(priv, vcore_addr + 4);
	return (datams << 32) | datals;
}

#endif

static void mscc_fa_vcore_writel(struct mscc_fa_fdma *priv, u64 vcore_addr, 
				u32 value) {
	unsigned long deadline;
	u32 status;

	/* Wait for not busy */
	deadline = jiffies + msecs_to_jiffies(VCORE_ACCESS_TIMEOUT_MS);
	do {
		status = mscc_fa_readl(priv, 
				       DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_OFF);
	} while (time_before(jiffies, deadline) &&
		 (status & DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_BUSY));
	/* Set VA_SIZE = 0 (32 bit access) */
	mscc_fa_writel(priv, DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_OFF, 0);
	/* Set LS Word */
	mscc_fa_writel(priv, DEVCPU_GCB_VCORE_ACCESS_VA_ADDR_LSB_OFF, 
		       (u32)vcore_addr);
	/* Set MS Word */
	mscc_fa_writel(priv, DEVCPU_GCB_VCORE_ACCESS_VA_ADDR_MSB_OFF, 
		       (u32)(vcore_addr >> 32));
	/* Write VA_DATA to trigger write operation */
	mscc_fa_writel(priv, DEVCPU_GCB_VCORE_ACCESS_VA_DATA_OFF, value);
	/* Wait while VA_BUSY is set, handle timeouts */
	deadline = jiffies + msecs_to_jiffies(VCORE_ACCESS_TIMEOUT_MS);
	do {
		status = mscc_fa_readl(priv, 
				       DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_OFF);
	} while (time_before(jiffies, deadline) &&
		 (status & DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_BUSY) &&
		 !(status & DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_ERR_M));
	if (status & DEVCPU_GCB_VCORE_ACCESS_VA_CTRL_ERR_M) {
		pr_err("%s:%d %s: Error writing VCore Access at 0x%llx", 
			__FILE__, __LINE__, __func__, vcore_addr);
	}
#ifdef FDMA_LOG
	pr_debug("%s:%d %s: VCore Access Write at 0x%llx, value 0x%08x", 
		__FILE__, __LINE__, __func__, vcore_addr, value);
#endif
}

/* Using Vcore access to read VCore address space */
static u32 mscc_fa_vcore_read_reg(struct mscc_fa_fdma *priv, u32 reg) {
	u64 vcore_addr = 
		priv->regions[GET_REGION_INDEX(reg)].vcore_addr + 
		GET_ADDRESS(reg);
	return mscc_fa_vcore_readl(priv, vcore_addr);
}

/* Using Vcore access to write VCore address space */
static void mscc_fa_vcore_write_reg(struct mscc_fa_fdma *priv, 
				    u32 reg, u32 value) {
	u64 vcore_addr = 
		priv->regions[GET_REGION_INDEX(reg)].vcore_addr + 
		GET_ADDRESS(reg);
	mscc_fa_vcore_writel(priv, vcore_addr, value);
}

#ifdef FDMA_DCB_LOG
static void mscc_fa_fdma_show_dcb_via_vcore(struct mscc_fa_fdma *priv, 
	struct mscc_fa_fdma_dcb *dcb)
{
	u64 vcore_addr = dcb->phys + dcb->offset;
	u64 val;
	int idx;

	pr_debug("%s:%d %s, txd: 0x%px\n", 
		__FILE__, __LINE__, __func__, 
		&dcb->txd);
	pr_debug("   dcb_phys: 0x%09llx\n", dcb->txd.phys);
	val = mscc_fa_vcore_readll(priv, vcore_addr); 
	vcore_addr += 8;
	pr_debug("    nextptr: 0x%09llx\n", val);
	val = mscc_fa_vcore_readll(priv, vcore_addr); 
	vcore_addr += 8;
	pr_debug("       info: 0x%09llx\n", val);
	for (idx = 0; idx < FDMA_DCB_MAX_DBS; ++idx) {
		val = mscc_fa_vcore_readll(priv, vcore_addr); 
		vcore_addr += 8;
		if (val != FDMA_DCB_INVALID_DATA) {
			pr_debug(" dataptr[%02d]: 0x%09llx\n", idx, val);
			val = mscc_fa_vcore_readll(priv, vcore_addr); 
			pr_debug("  status[%02d]: 0x%09llx\n", idx, val);
			vcore_addr += 8;
		} else {
			vcore_addr += 8;
		}
	}
}
#endif

#ifdef FDMA_DCB_ERROR_LOG
static struct mscc_fa_fdma_dcb *mscc_fa_fdma_get_previous_dcb(
	struct mscc_fa_fdma *priv, u32 chan)
{
	struct mscc_fa_fdma_dcb *dcb = 0;
	u64 dcb_phys = mscc_fa_readll(priv, 
		FDMA_DCB_LLP_PREV_R_OFF(chan), 
		FDMA_DCB_LLP_PREV1_R_OFF(chan));

	if  (dcb_phys > 1 && dcb_phys != -1) {
		dcb = phys_to_virt(dcb_phys) - FDMA_ATU_TARGET_SPACE;
	}
	return dcb;
}
#endif


#ifdef FDMA_DCB_LOG
static void mscc_fa_fdma_follow_queued_dcb_chain(
	struct mscc_fa_fdma_dcb *first, u32 chan)
{
	struct mscc_fa_fdma_dcb *dcb;
	struct dma_async_tx_descriptor *txd;
	u64 dcb_phys = virt_to_phys(first) + FDMA_ATU_TARGET_SPACE;
	int idx = 0;
	int jdx;

	pr_debug("%s:%d %s, Channel: %d\n", 
		__FILE__, __LINE__, __func__, 
		chan);
	while (dcb_phys > 1 && dcb_phys != -1) {
		dcb = phys_to_virt(dcb_phys) - FDMA_ATU_TARGET_SPACE;
		txd = &dcb->txd;
		pr_debug("        txd: 0x%px\n", txd);
		pr_debug("   dcb_phys: 0x%09llx\n", dcb->txd.phys);
		pr_debug("    nextptr: 0x%09llx\n", dcb->hw.nextptr);
		pr_debug("       info: 0x%09llx\n", dcb->hw.info);
		for (jdx = 0; jdx < FDMA_DCB_MAX_DBS; ++jdx) {
			if (dcb->hw.block[jdx].dataptr != 
			    FDMA_DCB_INVALID_DATA) {
				pr_debug(" dataptr[%02d]: 0x%09llx\n", 
					jdx, dcb->hw.block[jdx].dataptr);
				pr_debug("  status[%02d]: 0x%09llx\n", 
					jdx, dcb->hw.block[jdx].status);
			}
		}
		++idx;
		dcb_phys = dcb->hw.nextptr;
	}
	pr_debug("%s:%d %s, Channel: %d, end, total: %u\n",
		__FILE__, __LINE__, __func__, chan, idx);
}
#endif

static inline struct mscc_fa_fdma *to_mscc_fa_fdma(struct dma_device *dd)
{
	return container_of(dd, struct mscc_fa_fdma, dma);
}

static inline struct mscc_fa_fdma_channel *to_mscc_fa_fdma_channel(
	struct dma_chan *c)
{
	return container_of(c, struct mscc_fa_fdma_channel, chan);
}

static void mscc_fa_fdma_free_dcb_list(struct dma_chan *chan)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	struct mscc_fa_fdma_channel *fdma_chan = to_mscc_fa_fdma_channel(chan);
	struct mscc_fa_fdma_dcb *iter;

	list_for_each_entry(iter, &fdma_chan->queued_dcbs, node) {
		pr_debug("%s:%d %s: Channel: %d, DCB: 0x%llx:"
			" state: %u, cookie: %u\n", 
			__FILE__, __LINE__, __func__, 
			fdma_chan->chan.chan_id,
			iter->phys,
			iter->state,
			iter->txd.cookie);
		dma_pool_free(priv->dcb_pool, iter, iter->phys);
	}
}

static int mscc_fa_fdma_wait_for_xtr_buffer_empty(struct mscc_fa_fdma *priv)
{
	unsigned long deadline;
	int cpuport = 0; /* Using CPU port 0 */
	int empty;

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	/* Wait here until the extraction buffer is empty */
	deadline = jiffies + msecs_to_jiffies(FDMA_DISABLE_TIMEOUT_MS);
	do {
		empty = mscc_fa_readl(priv, 
			FDMA_PORT_CTRL_R_OFF(cpuport)) & 
			FDMA_PORT_CTRL_XTR_BUF_IS_EMPTY;
		pr_debug("%s:%d %s: empty: %u\n", 
			__FILE__, __LINE__, __func__, empty);
	} while (time_before(jiffies, deadline) && (empty == 0)) ;

	return empty;
}

static void mscc_fa_fdma_start(struct dma_chan *chan)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	int cpuport = 0; /* Using CPU port 0 */
	u32 control;

	pr_debug("%s:%d %s: Channel: %u\n", __FILE__, __LINE__, __func__, 
		chan->chan_id);
	priv->chans[chan->chan_id].state = DCS_ACTIVE;
	if (chan->chan_id >= DMA_RX_CHANNEL) {
		/* Start extraction */
		control = mscc_fa_readl(priv, 
					FDMA_PORT_CTRL_R_OFF(cpuport));
		control &= ~FDMA_PORT_CTRL_XTR_STOP;
		mscc_fa_writel(priv, FDMA_PORT_CTRL_R_OFF(cpuport), control);
	} else {
		/* Start injection */
		control = mscc_fa_readl(priv, 
					FDMA_PORT_CTRL_R_OFF(cpuport));
		control &= ~FDMA_PORT_CTRL_INJ_STOP;
		mscc_fa_writel(priv, FDMA_PORT_CTRL_R_OFF(cpuport), control);
	}

}

static int mscc_fa_fdma_stop(struct dma_chan *chan)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	int cpuport = 0; /* Using CPU port 0 */
	int stopped = 0;
	u32 control, empty;

	pr_debug("%s:%d %s: Channel: %u\n", __FILE__, __LINE__, __func__, 
		chan->chan_id);
	priv->chans[chan->chan_id].state = DCS_STOPPING;
	if (chan->chan_id >= DMA_RX_CHANNEL) {
		empty = mscc_fa_fdma_wait_for_xtr_buffer_empty(priv);
		if (!empty) {
			return stopped;
		}
		/* Stop extraction */
		control = mscc_fa_readl(priv, 
					FDMA_PORT_CTRL_R_OFF(cpuport));
		control |= FDMA_PORT_CTRL_XTR_STOP;
		mscc_fa_writel(priv, FDMA_PORT_CTRL_R_OFF(cpuport), control);
		stopped = mscc_fa_fdma_wait_for_xtr_buffer_empty(priv);
	} else {
		/* Stop injection */
		control = mscc_fa_readl(priv, 
					FDMA_PORT_CTRL_R_OFF(cpuport));
		control |= FDMA_PORT_CTRL_INJ_STOP;
		mscc_fa_writel(priv, FDMA_PORT_CTRL_R_OFF(cpuport), control);
		stopped = 1;
	}
	return stopped;

}

static int mscc_fa_fdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	u32 chan_mask;

	pr_debug("%s:%d %s: Channel: %u\n", __FILE__, __LINE__, __func__, 
		chan->chan_id);
	
	mscc_fa_fdma_start(chan);

	dma_cookie_init(chan);
	/* Enable DB interrupts */
	chan_mask = mscc_fa_readl(priv, FDMA_INTR_DB_ENA_OFF) 
		| BIT(chan->chan_id);
	spin_lock(&priv->lock);
	mscc_fa_writel(priv, FDMA_INTR_DB_ENA_OFF, chan_mask);
	spin_unlock(&priv->lock);
	return 0;
}

static void mscc_fa_fdma_free_chan_resources(struct dma_chan *chan)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	u32 chan_mask;

	pr_debug("%s:%d %s: Channel: %u\n", __FILE__, __LINE__, __func__, 
		chan->chan_id);

	mscc_fa_fdma_stop(chan);

	/* Disable the channel */
	spin_lock(&priv->lock);
	mscc_fa_writel(priv, FDMA_CH_DISABLE_OFF, ~BIT(chan->chan_id));
	spin_unlock(&priv->lock);

	/* Disable the channels DB interrupt */
	chan_mask = mscc_fa_readl(priv, FDMA_INTR_DB_ENA_OFF) & 
		~BIT(chan->chan_id);
	spin_lock(&priv->lock);
	mscc_fa_writel(priv, FDMA_INTR_DB_ENA_OFF, chan_mask);
	spin_unlock(&priv->lock);
}


static dma_cookie_t mscc_fa_fdma_tx_submit(struct dma_async_tx_descriptor *txd)
{
	if (txd) {
		pr_debug("%s:%d %s: Channel: %u", 
			__FILE__, __LINE__, __func__,
			txd->chan->chan_id);
		return dma_cookie_assign(txd);
	} else {
		pr_err("%s:%d %s: Invalid TXD", 
			__FILE__, __LINE__, __func__);
		return 0;
	}
}

static struct dma_async_tx_descriptor *
mscc_fa_fdma_prep_slave_sg(struct dma_chan *chan, 
	struct scatterlist *sgl,
	unsigned int sg_len,
	enum dma_transfer_direction direction,
	unsigned long flags, 
	void *context)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	struct mscc_fa_fdma_channel *fdma_chan = to_mscc_fa_fdma_channel(chan);
	struct scatterlist *sg;
	struct mscc_fa_fdma_dcb *first = 0;
	struct mscc_fa_fdma_dcb *dcb = 0;
	int sidx; /* Segment index */
	int idx;  /* Block index */

	if (!sgl) {
		return NULL;
	}

	if (!is_slave_direction(direction)) {
		dev_err(&chan->dev->device, "Invalid DMA direction\n");
		return NULL;
	}
	if (fdma_chan->state == DCS_STOPPING) {
		pr_debug("%s:%d %s, Stopping channel %d\n", 
			__FILE__, __LINE__, __func__, chan->chan_id);
		return NULL;
	}

	dev_dbg(&chan->dev->device, "%s: sg_len=%d, dir=%s,"
		" flags=0x%lx, elems=%d\n",
		__func__, sg_len,
		direction == DMA_MEM_TO_DEV ? "to device" : "from device",
		flags, sg_nents(sgl));

	spin_lock(&priv->lock);
	for_each_sg(sgl, sg, sg_len, sidx) {
		u64 len, off;
		dma_addr_t db_phys = 0;
		/* Interrupt on DB status update */
		u32 status_flags = FDMA_DCB_STATUS_INTR;
		idx = sidx % FDMA_DCB_MAX_DBS;

		/* One DCB has room for FDMA_DCB_MAX_DDBS blocks */
		if (idx == 0) {
			int jdx;

			if (list_empty(&priv->free_dcbs)) {
				pr_err("%s:%d %s: no more DCBs\n", 
					__FILE__, __LINE__, __func__);
				return NULL;
			}
			dcb = list_first_entry(&priv->free_dcbs, 
					       struct mscc_fa_fdma_dcb, node);
			memset(dcb, 0, sizeof(struct  mscc_fa_fdma_dcb_hw));
			memset(dcb->binfo, 0, 
			       sizeof(struct  mscc_fa_fdma_block_info));
			dcb->state = DCBS_QUEUED;
			dcb->valid_blocks = 0;
			dcb->is_last_dcb = 0;
			/* No next DCB */
			dcb->hw.nextptr = FDMA_DCB_INVALID_DATA;
			for (jdx = 0; jdx < FDMA_DCB_MAX_DBS; ++jdx) {
				dcb->hw.block[jdx].dataptr = 
					FDMA_DCB_INVALID_DATA;
			}
			dma_async_tx_descriptor_init(&dcb->txd, chan);
			dcb->txd.tx_submit = mscc_fa_fdma_tx_submit;
			/* Convert to PCIE address space */
			dcb->txd.phys = dcb->phys + dcb->offset;
			/* Move item into the channel */
			list_move_tail(&dcb->node, &fdma_chan->queued_dcbs);
		}
		db_phys = sg_dma_address(sg);
		/* Convert to PCIe address space */
		db_phys += dcb->offset;
		off = db_phys & 0x7;
		db_phys &= ~0x7;
		len = sg_dma_len(sg);
		dcb->binfo[idx].size = len;
		if (direction == DMA_MEM_TO_DEV) {
			if (sidx == 0) {
				status_flags |= FDMA_DCB_STATUS_SOF;
			}
			if (sg_is_last(sg)) {
				status_flags |= FDMA_DCB_STATUS_EOF;
			}
			dcb->hw.block[idx].dataptr = db_phys;
			dcb->hw.block[idx].status =
				FDMA_DCB_STATUS_BLOCKL(len) | 
				status_flags |
				FDMA_DCB_STATUS_BLOCKO(off);
		} else {
			/* Length is a multipla of 128 */
			dcb->hw.info = FDMA_DCB_INFO_DATAL(
				len & ~FDMA_BUFFER_MASK);
			dcb->hw.block[idx].dataptr = db_phys;
			dcb->hw.block[idx].status = status_flags;
		}
		dcb->valid_blocks++;
		if (!first) {
			first = dcb;
		}
		pr_debug("%s:%d %s: txd: 0x%px, DCB: 0x%llx, Block[%02d], "
			"dataptr: 0x%09llx, offset: 0x%llu, bytes: %llu\n",
			__FILE__, __LINE__, __func__,
			&first->txd,
			dcb->txd.phys,
			idx, dcb->hw.block[idx].dataptr,
			off, len);
	}
	dcb->is_last_dcb = 1;
	dcb->first_dcb = first;
	spin_unlock(&priv->lock);
	return &first->txd;
}

static enum dma_status mscc_fa_fdma_tx_status(struct dma_chan *chan, 
	dma_cookie_t cookie,
	struct dma_tx_state *txstate)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	struct mscc_fa_fdma_channel *fdma_chan;
	struct mscc_fa_fdma_dcb *iter;
	enum dma_status status;
	int found = 0;
	unsigned int residue = 0;
	int idx;

	pr_debug("%s:%d %s, cookie: %u\n", 
		__FILE__, __LINE__, __func__, cookie);

	status = dma_cookie_status(chan, cookie, txstate);

	fdma_chan = &priv->chans[chan->chan_id];

	if (fdma_chan->state == DCS_ERROR) {
		return DMA_ERROR;
	}
	if (status != DMA_IN_PROGRESS) {
		return status;
	}
	list_for_each_entry(iter, &fdma_chan->queued_dcbs, node) {
		pr_debug("%s:%d %s: Channel: %d, DCB: 0x%llx:"
			" state: %u, cookie: %u\n", 
			__FILE__, __LINE__, __func__, 
			fdma_chan->chan.chan_id,
			iter->txd.phys,
			iter->state,
			iter->txd.cookie);
		if (iter->txd.cookie == cookie) {
			found = 1;
			for (idx = 0; idx < FDMA_DCB_MAX_DBS; ++idx) {
				residue += iter->binfo[idx].size;
			}
		} else if (iter->txd.cookie > DMA_MIN_COOKIE) {
			found = 0;
		} else if (found) {
			for (idx = 0; idx < FDMA_DCB_MAX_DBS; ++idx) {
				residue += iter->binfo[idx].size;
			}
		}
	}
	txstate->residue = residue;
	return status;
}

static void mscc_fa_fdma_issue_pending(struct dma_chan *chan)
{
	u32 channel_bit = BIT(chan->chan_id);
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	struct mscc_fa_fdma_dcb *dcb, *first = 0;
	struct mscc_fa_fdma_channel *fdma_chan;
	int idx = 0;
	int queued = 0;

	pr_debug("%s:%d %s, Channel: %d\n", 
		__FILE__, __LINE__, __func__, 
		chan->chan_id);
	spin_lock(&priv->lock);
	fdma_chan = &priv->chans[chan->chan_id];
	list_for_each_entry(dcb, &fdma_chan->queued_dcbs, node) {
		if (dcb->state == DCBS_QUEUED) {
			if (first == 0) {
				first = dcb;
			}
			++queued;
		}
		++idx;
	}
	spin_unlock(&priv->lock);
	if (!first) {
		pr_err("%s:%d %s, Channel: %d, nothing queued\n", 
			__FILE__, __LINE__, __func__, 
			chan->chan_id);
		return;
	}
	pr_debug("%s:%d %s, Channel: %d, state: %u, len: %d, queued: %d\n", 
		__FILE__, __LINE__, __func__, 
		chan->chan_id, fdma_chan->state, idx, queued);

	switch (fdma_chan->state) {
	case DCS_ACTIVE: {
		struct mscc_fa_fdma_dcb *prev = 0;
		int idx = 0;

		pr_debug("%s:%d %s, Activate channel %d, DCB: 0x%llx\n", 
			__FILE__, __LINE__, __func__, chan->chan_id, 
			first->txd.phys);
		spin_lock(&priv->lock);
		list_for_each_entry(dcb, &fdma_chan->queued_dcbs, node) {
			if (dcb->state == DCBS_QUEUED && prev) {
				pr_debug("%s:%d %s, Channel: %d: chain[%02d]:"
					" DCB: 0x%llx -> 0x%llx\n", 
					__FILE__, __LINE__, __func__,
					chan->chan_id,
					idx,
					prev->txd.phys, dcb->txd.phys);
				prev->hw.nextptr = dcb->txd.phys; /* Valid */
#ifdef FDMA_DCB_LOG
				mscc_fa_fdma_show_dcb_via_vcore(priv, prev);
#endif
			}
			prev = dcb;
			++idx;
		}
#ifdef FDMA_DCB_LOG
		mscc_fa_fdma_follow_queued_dcb_chain(first, chan->chan_id);
#endif
		spin_unlock(&priv->lock);
		/* Write the DCB address */
		mscc_fa_writell(priv, 
				FDMA_DCB_LLP_R_OFF(chan->chan_id), 
				FDMA_DCB_LLP1_R_OFF(chan->chan_id), 
				first->txd.phys);
		/* Activate the channel */
		mscc_fa_writel(priv, FDMA_CH_ACTIVATE_OFF, channel_bit);
		fdma_chan->state = DCS_RUNNING;
		break;
	}
	case DCS_RUNNING: {
		struct mscc_fa_fdma_dcb *prev = 0;
		int idx = 0;
#ifdef FDMA_DCB_ERROR_LOG
		struct mscc_fa_fdma_dcb *llp_prev = 
			mscc_fa_fdma_get_previous_dcb(priv, chan->chan_id);
#endif

		spin_lock(&priv->lock);
		list_for_each_entry(dcb, &fdma_chan->queued_dcbs, node) {
			if (dcb->state == DCBS_QUEUED && prev) {
				prev->hw.nextptr = dcb->txd.phys; /* Valid */
				pr_debug("%s:%d %s, Channel: %d: chain[%02d]:"
					" DCB: 0x%llx -> 0x%llx\n", 
					__FILE__, __LINE__, __func__,
					chan->chan_id,
					idx,
					prev->txd.phys, dcb->txd.phys);
#ifdef FDMA_DCB_LOG
				mscc_fa_fdma_show_dcb_via_vcore(priv, prev);
#endif
			}
			prev = dcb;
			++idx;
		}
		spin_unlock(&priv->lock);
		pr_debug("%s:%d %s, Reload channel %d\n", 
			__FILE__, __LINE__, __func__, chan->chan_id);
		mscc_fa_writel(priv, FDMA_CH_RELOAD_OFF, channel_bit);
		break;
	}
	case DCS_STOPPING:
		pr_debug("%s:%d %s, Stopping channel %d\n", 
			__FILE__, __LINE__, __func__, chan->chan_id);
		break;
	case DCS_IDLE:
		/* When is a reload needed ? */
		pr_debug("%s:%d %s, Queue channel %d, DCB: 0x%llx\n", 
			__FILE__, __LINE__, __func__, chan->chan_id, 
			dcb->txd.phys);
		break;
	case DCS_ERROR:
		pr_err("%s:%d %s, Errored channel %d,\n", 
			__FILE__, __LINE__, __func__, chan->chan_id);
		break;
	}
}

static void mscc_fa_fdma_notify_clients_abort(struct dma_chan *chan)
{
	struct mscc_fa_fdma_channel *fdma_chan = to_mscc_fa_fdma_channel(chan);
	struct mscc_fa_fdma_dcb *iter;
	struct dmaengine_result dma_result = {
		.result = DMA_TRANS_ABORTED,
		.residue = 0
	};
	int idx;

	pr_debug("%s:%d %s: Channel: %d, begin\n", 
		__FILE__, __LINE__, __func__,
		fdma_chan->chan.chan_id);

	list_for_each_entry(iter, &fdma_chan->queued_dcbs, node) {
		pr_debug("%s:%d %s: Channel: %d, DCB: 0x%llx:"
			" state: %u, cookie: %u\n", 
			__FILE__, __LINE__, __func__, 
			fdma_chan->chan.chan_id,
			iter->txd.phys,
			iter->state,
			iter->txd.cookie);
		for (idx = 0; idx < FDMA_DCB_MAX_DBS; ++idx) {
			iter->binfo[idx].size = 0;
		}
		if (iter->txd.cookie > DMA_MIN_COOKIE) {
			/* Requests have valid cookies */
			fdma_chan->tx_state.used = iter->txd.cookie;
			fdma_chan->tx_state.last = iter->txd.cookie;
			fdma_chan->tx_state.residue = 0;
			iter->state = DCBS_COMPLETE;
			dma_cookie_complete(&iter->txd);
			pr_debug("%s:%d %s: Channel: %d, notify client abort:"
				" txd: 0x%px\n", 
				__FILE__, __LINE__, __func__,
				fdma_chan->chan.chan_id,
				&iter->txd);
			dmaengine_desc_get_callback_invoke(&iter->txd, 
							   &dma_result);
		}
	}
	pr_debug("%s:%d %s: Channel: %d, end\n", 
		__FILE__, __LINE__, __func__,
		fdma_chan->chan.chan_id);
}

static void mscc_fa_fdma_sync(struct dma_chan *chan)
{
	struct mscc_fa_fdma *priv = to_mscc_fa_fdma(chan->device);
	u32 channel_mask = BIT(chan->chan_id);
	u32 status;
	unsigned long deadline;

	pr_debug("%s:%d %s: Channel: %u\n", __FILE__, __LINE__, __func__, 
		chan->chan_id);
	/* Wait here until the FDMA has stopped */
	deadline = jiffies + msecs_to_jiffies(FDMA_DISABLE_TIMEOUT_MS);
	do {
		status = mscc_fa_readl(priv, FDMA_CH_ACTIVE_OFF);
		status |= mscc_fa_readl(priv, FDMA_CH_PENDING_OFF);
		pr_debug("%s:%d %s: Channel: %u, status: %u\n", 
			__FILE__, __LINE__, __func__, 
			chan->chan_id, status);
	} while (time_before(jiffies, deadline) && (status & channel_mask)) ;

	/* Notify the client that the queued transfers have been aborted */
	mscc_fa_fdma_notify_clients_abort(chan);
}

static int mscc_fa_fdma_terminate(struct dma_chan *chan)
{
	pr_debug("%s:%d %s: Channel: %u\n", __FILE__, __LINE__, __func__, 
		chan->chan_id);
	return 0;
}

static void mscc_fa_fdma_handle_error(struct mscc_fa_fdma_channel *fdma_chan, 
	struct mscc_fa_fdma_dcb *dcb)
{
	struct dmaengine_result dma_result = {
		.result = DMA_TRANS_ABORTED,
		.residue = 0
	};
	struct mscc_fa_fdma_dcb *iter;
	struct mscc_fa_fdma_dcb *first = dcb->first_dcb;

	pr_err("%s:%d %s: DCB: 0x%llx: error: inform client\n", 
		__FILE__, __LINE__, __func__, dcb->txd.phys);

	dma_result.result = DMA_TRANS_WRITE_FAILED;
	if (fdma_chan->chan.chan_id >= DMA_RX_CHANNEL) {
		dma_result.result = DMA_TRANS_READ_FAILED;
	}
	
	/* Mark all DCBs in the request as completed */
	list_for_each_entry(iter, &fdma_chan->queued_dcbs, node) {
		iter->state = DCBS_COMPLETE;
		if (iter == dcb) {
			break;
		}
	}

	dma_cookie_complete(&first->txd);
	fdma_chan->tx_state.last = first->txd.cookie;
	dmaengine_desc_get_callback_invoke(&first->txd, &dma_result);
}

static u32 mscc_fa_fdma_update_completion(
	struct mscc_fa_fdma_channel *fdma_chan, 
	struct mscc_fa_fdma_dcb *request,
	struct mscc_fa_fdma_dcb *from,
	int from_index,
	struct mscc_fa_fdma_dcb *to,
	int to_index)
{
	struct mscc_fa_fdma_dcb *iter = from;
	int idx = 0, max;
	u32 packet_size = 0;
	u32 residue = 0;

	pr_debug("%s:%d %s: Channel: %d, begin: "
		"From: 0x%llx [%02d], To: 0x%llx [%02d]\n", 
		__FILE__, __LINE__, __func__,
		fdma_chan->chan.chan_id,
		from->txd.phys,
		from_index,
		to->txd.phys,
		to_index);

	/* Mark blocks as transferred.  Complete DCBs and requests */
	list_for_each_entry_from(iter, &fdma_chan->queued_dcbs, node) {
		pr_debug("%s:%d %s: Channel: %d, Marking: DCB: 0x%llx,"
			" state: %u, valid_blocks: %u\n", 
			__FILE__, __LINE__, __func__,
			fdma_chan->chan.chan_id, 
			iter->txd.phys,
			iter->state,
			iter->valid_blocks);
		if (iter->state != DCBS_QUEUED) {
			continue;
		}
		idx = 0;
		max = iter->valid_blocks;
		if (iter == from) {
			idx = from_index;
		}
		if (iter == to) {
			max = to_index;
		}
		for (; idx < max; ++idx) {
			pr_debug("%s:%d %s: Channel: %d, Marking, DCB: 0x%llx,"
				" Block[%02d], dataptr: 0x%09llx,"
				" status: 0x%09llx, bytes: %u\n", 
				__FILE__, __LINE__, __func__,
				fdma_chan->chan.chan_id, 
				iter->txd.phys,
				idx,
				iter->hw.block[idx].dataptr,
				iter->hw.block[idx].status,
				iter->binfo[idx].size);
			/* Remove transferred data from the residue */
			packet_size += iter->binfo[idx].size;
			iter->binfo[idx].size = 0;
		}
		if (iter->valid_blocks == idx) {
			/* Last block in this DCB has been transferred  */
			pr_debug("%s:%d %s: Channel: %d, completed DCB:"
				" 0x%llx\n", 
				__FILE__, __LINE__, __func__,
				fdma_chan->chan.chan_id, iter->txd.phys);
			iter->state = DCBS_COMPLETE;
			/* Last DCB in this request has been transferred  */
			if (iter->is_last_dcb) {
				fdma_chan->tx_state.last = 
					iter->first_dcb->txd.cookie;
				dma_cookie_complete(&iter->first_dcb->txd);
				pr_debug("%s:%d %s: Channel: %d,"
					" completed cookie: %u\n", 
					__FILE__, __LINE__, __func__,
					fdma_chan->chan.chan_id, 
					fdma_chan->tx_state.last);
			}
		}
		if (iter == to) {
			break;
		}
	}
	/* Continue to count the residue if not at the end of the request */
	if (!(idx == iter->valid_blocks && iter->is_last_dcb)) {
		list_for_each_entry_from(iter, &fdma_chan->queued_dcbs, node) {
			pr_debug("%s:%d %s: Channel: %d, Calculating: DCB:"
				" 0x%llx, state: %u, valid_blocks: %u\n", 
				__FILE__, __LINE__, __func__,
				fdma_chan->chan.chan_id, 
				iter->txd.phys,
				iter->state,
				iter->valid_blocks);
			if (iter->state != DCBS_QUEUED) {
				continue;
			}
			idx = 0;
			if (iter == to) {
				idx = to_index;
			}
			for (; idx < iter->valid_blocks; ++idx) {
				pr_debug("%s:%d %s: Channel: %d, Calculating,"
					" DCB: 0x%llx, Block[%02d], dataptr:"
					" 0x%09llx, status: 0x%09llx,"
					" bytes: %u\n", 
					__FILE__, __LINE__, __func__,
					fdma_chan->chan.chan_id, 
					iter->txd.phys,
					idx,
					iter->hw.block[idx].dataptr,
					iter->hw.block[idx].status,
					iter->binfo[idx].size);
				residue += iter->binfo[idx].size;
			}
			idx = 0;
			if (iter->is_last_dcb) {
				break;
			}
		}
	}

	pr_debug("%s:%d %s: Channel: %d, end, packet size: %u, residue: %u\n", 
		__FILE__, __LINE__, __func__,
		fdma_chan->chan.chan_id, packet_size, residue);
	return residue;
}

static void mscc_fa_fdma_notify_clients(struct mscc_fa_fdma_channel *fdma_chan)
{
	struct mscc_fa_fdma_dcb *request = 0;
	struct mscc_fa_fdma_dcb *iter, *first = 0;
	int idx, first_block = 0;
	struct dmaengine_result dma_result = {
		.result = DMA_TRANS_ABORTED,
		.residue = 0
	};

	pr_debug("%s:%d %s: Channel: %d, begin\n", 
		__FILE__, __LINE__, __func__,
		fdma_chan->chan.chan_id);

	list_for_each_entry(iter, &fdma_chan->queued_dcbs, node) {
		pr_debug("%s:%d %s: Channel: %d, DCB: 0x%llx: state: %u,"
			" cookie: %u\n", 
			__FILE__, __LINE__, __func__, 
			fdma_chan->chan.chan_id,
			iter->txd.phys,
			iter->state,
			iter->txd.cookie);
		if (iter->txd.cookie > DMA_MIN_COOKIE) {
			/* Requests have valid cookies */
			if (!request) {
				/* First in queue is the used TXD */
				fdma_chan->tx_state.used = iter->txd.cookie;
			}
			request = iter;
		}
		for (idx = 0; idx < iter->valid_blocks; ++idx) {
			u32 residue;

			pr_debug("%s:%d %s: Channel: %d, DCB: 0x%llx,"
				" Block[%02d], dataptr: 0x%09llx, status:"
				" 0x%09llx: bytes: %u\n", 
				__FILE__, __LINE__, __func__,
				fdma_chan->chan.chan_id, 
				iter->txd.phys,
				idx,
				iter->hw.block[idx].dataptr,
				iter->hw.block[idx].status,
				iter->binfo[idx].size);
			if (iter->binfo[idx].size == 0) {
				continue;
			}
			if (!(iter->hw.block[idx].status & 
			      FDMA_DCB_STATUS_DONE)) {
				break;
			}
			if (iter->hw.block[idx].status & FDMA_DCB_STATUS_SOF) {
				first = iter;
				first_block = idx;
			}
			if (!(iter->hw.block[idx].status & 
			      FDMA_DCB_STATUS_EOF)) {
				continue;
			}
			if (!request) {
				break;
			}
			residue = mscc_fa_fdma_update_completion(
				fdma_chan, request,
				first, first_block, 
				iter, idx + 1);
			fdma_chan->tx_state.residue = residue;
			dma_result.residue = residue;
			dma_result.result = DMA_TRANS_NOERROR;
			pr_debug("%s:%d %s: Channel: %d, notify client:"
				" txd: 0x%px, residue: %u\n", 
				__FILE__, __LINE__, __func__,
				fdma_chan->chan.chan_id,
				&request->txd,
				residue);
			dmaengine_desc_get_callback_invoke(
				&request->txd, &dma_result);
		}
	}
	pr_debug("%s:%d %s: Channel: %d, end\n", 
		__FILE__, __LINE__, __func__,
		fdma_chan->chan.chan_id);
}

static void mscc_fa_fdma_tasklet(unsigned long data)
{
	struct mscc_fa_fdma *priv = (struct mscc_fa_fdma *)data;
	struct mscc_fa_fdma_dcb *dcb;
	struct mscc_fa_fdma_channel *fdma_chan;
	struct dma_async_tx_descriptor *txd;
	u32 chan;

	pr_debug("%s:%d %s: begin\n", __FILE__, __LINE__, __func__);
	for (chan = 0; chan < priv->nr_pchans; ++chan) {
		int idx = 0;
		struct mscc_fa_fdma_dcb *tmp;

		fdma_chan = &priv->chans[chan];
		if (fdma_chan->state == DCS_IDLE) {
			pr_debug("%s:%d %s: Channel: %u, idle\n", 
				__FILE__, __LINE__, __func__, chan);
			continue;
		}
		fdma_chan->completed_dcbs = 0;
		fdma_chan->queue_state_dcbs = 0;
		mscc_fa_fdma_notify_clients(fdma_chan);
		list_for_each_entry_safe(dcb, tmp, 
					 &fdma_chan->queued_dcbs, node) {
			txd = &dcb->txd;
			switch (dcb->state) {
			case DCBS_IDLE:
				spin_lock(&priv->lock);
				list_move_tail(&dcb->node, &priv->free_dcbs);
				spin_unlock(&priv->lock);
				break;
			case DCBS_QUEUED:
				++fdma_chan->queue_state_dcbs;
				break;
			case DCBS_COMPLETE:
				++fdma_chan->completed_dcbs;
				break;
			case DCBS_ERROR:
				fdma_chan->state = DCS_ERROR;
				mscc_fa_fdma_handle_error(fdma_chan, dcb);
				break;
			}
			++idx;
		}
	}
	pr_debug("%s:%d %s: channel status\n", __FILE__, __LINE__, __func__);
	for (chan = 0; chan < priv->nr_pchans; ++chan) {
		int idx = 0;

		fdma_chan = &priv->chans[chan];
		if (fdma_chan->state == DCS_IDLE) {
			pr_debug("%s:%d %s: Channel: %u, idle\n", 
				__FILE__, __LINE__, __func__, chan);
			continue;
		}
		if (fdma_chan->completed_dcbs >= 2) {
			list_for_each_entry(dcb, 
					    &fdma_chan->queued_dcbs, node) {
				if (fdma_chan->completed_dcbs >= 2) {
					if (dcb->state == DCBS_COMPLETE) {
						dcb->state = DCBS_IDLE;
						--fdma_chan->completed_dcbs;
					}
				} else {
					break;
				}
			}
		}
		idx = 0;
		list_for_each_entry(dcb, &fdma_chan->queued_dcbs, node) {
			pr_debug("%s:%d %s: channel %u: idx: %02d,"
				" state: %u, DCB: 0x%llx\n", 
				__FILE__, __LINE__, __func__, 
				chan, idx, dcb->state, dcb->txd.phys);
			++idx;
		}
		pr_debug("%s:%d %s: channel %u: queued: %u\n", 
			__FILE__, __LINE__, __func__, chan,
			fdma_chan->queue_state_dcbs);
	}
	{
		int count = 0;
		struct list_head *node;

		list_for_each(node, &priv->free_dcbs) {
			++count;
		}
		pr_debug("%s:%d %s: end: free dcbs: %d\n", 
			__FILE__, __LINE__, __func__, count);
	}
}


static irqreturn_t mscc_fa_fdma_interrupt(int irq, void *dev_id)
{
	struct mscc_fa_fdma *priv = dev_id;
	u32 dcb = 0, db = 0, err = 0;

	pr_debug("%s:%d %s: begin\n", 
		__FILE__, __LINE__, __func__);
	dcb = mscc_fa_readl(priv, FDMA_INTR_DCB_OFF);
	db = mscc_fa_readl(priv, FDMA_INTR_DB_OFF);
	err = mscc_fa_readl(priv, FDMA_INTR_ERR_OFF);
	/* Clear interrupt */
	if (dcb) {
		mscc_fa_writel(priv, FDMA_INTR_DCB_OFF, dcb);
		pr_debug("%s:%d %s: DCB int: 0x%x\n", 
			__FILE__, __LINE__, __func__, dcb);
	}
	if (db) {
		mscc_fa_writel(priv, FDMA_INTR_DB_OFF, db);
		pr_debug("%s:%d %s: DB int: 0x%x\n", 
			__FILE__, __LINE__, __func__, db);
		tasklet_schedule(&priv->tasklet);
	}
	if (err) {
		u32 err_type = mscc_fa_readl(priv, FDMA_ERRORS_OFF);

		pr_err("%s:%d %s: ERR int: 0x%x\n", 
			__FILE__, __LINE__, __func__, err);
		pr_err("%s:%d %s: errtype: 0x%x\n", 
			__FILE__, __LINE__, __func__,
			err_type);
		mscc_fa_writel(priv, FDMA_INTR_ERR_OFF, err);
		mscc_fa_writel(priv, FDMA_ERRORS_OFF, err_type);
	}

	pr_debug("%s:%d %s: end\n", 
		__FILE__, __LINE__, __func__);

	return IRQ_HANDLED;
}

static int mscc_fa_access_test(struct mscc_fa_fdma *priv)
{
	/* Check VCore Access functionality by reading chip id */
	u32 test0 = 0x12345678;
	u32 test1 = 0xabcdef;
	u32 value = 0;
	struct mscc_fa_fdma_dcb_hw *dcb;
	void *dma_buffer;
	phys_addr_t phys = 0;
	u64 vcore_addr = FDMA_ATU_TARGET_SPACE;

	u32 chip_id = mscc_fa_vcore_read_reg(priv, DEVCPU_GCB_CHIP_REGS_ID_OFF);
	if (chip_id != 0x07568445) {
		pr_err("%s:%d %s: VCore access error: CSR space\n",
			__FILE__, __LINE__, __func__);
	}
	/* Write value to General CPU reg 0 */
	mscc_fa_vcore_write_reg(priv, CPU_REGS_GPR_R_OFF(0), test0);
	/* Write value to General CPU reg 1 */
	mscc_fa_vcore_write_reg(priv, CPU_REGS_GPR_R_OFF(1), test1);
	/* Read General CPU reg 0 */
	value = mscc_fa_vcore_read_reg(priv, CPU_REGS_GPR_R_OFF(0));
	if (test0 != value) {
		pr_err("%s:%d %s: VCore access error: AMBA_TOP space\n",
			__FILE__, __LINE__, __func__);
		return -1;
	}
	/* Read General CPU reg 1 */
	value = mscc_fa_vcore_read_reg(priv, CPU_REGS_GPR_R_OFF(1));
	if (test1 != value) {
		pr_err("%s:%d %s: VCore access error: AMBA_TOP space\n",
			__FILE__, __LINE__, __func__);
		return -1;
	}

	dma_buffer = devm_kmalloc(priv->dma.dev, 1024, GFP_KERNEL);
	phys = virt_to_phys(dma_buffer);
	dcb = dma_buffer;
	dcb->nextptr = 0x12345678abcdef;
	pr_debug("%s:%d %s: Allocation 0x%px at phys: 0x%px\n", 
		__FILE__, __LINE__, __func__, dma_buffer, (void *)phys);
	vcore_addr += phys;
	pr_debug("%s:%d %s: Address in VCore Space: 0x%px\n", 
		__FILE__, __LINE__, __func__, (void *)vcore_addr);

	value = mscc_fa_vcore_readl(priv, vcore_addr);
	if (0x78abcdef != value) {
		pr_err("%s:%d %s: VCore PCIe space access error:"
		       " AMBA_TOP space\n", 
			__FILE__, __LINE__, __func__);
		return -1;
	}
	value = mscc_fa_vcore_readl(priv, vcore_addr + 4);
	if (0x00123456 != value) {
		pr_err("%s:%d %s: VCore PCIe space access error:"
		       " AMBA_TOP space\n",
			__FILE__, __LINE__, __func__);
		return -1;
	}
	pr_debug("%s:%d %s: Successfully accessed host memory\n", 
		__FILE__, __LINE__, __func__);
	return 0;
}

static void mscc_fa_setup_pcie_atu(struct mscc_fa_fdma *priv)
{
	/* DMA Address Translation Configuration
	 * Enable outbound ATU for region 0: 
	 * Set enable bit
	 * Set base address = PCI_TARGET_SPACE (the vcore side)
	 * Set limit address =  PCI_TARGET_SPACE + 4GB (the vcore side)
	 * Set target address = 0 (the host side)
	 */
	mscc_fa_vcore_write_reg(priv, 
		PCIE_DM_EP_PF0_ATU_REGION_CTRL_2_OFF_OUTBOUND_0_OFF,
		PCIE_DM_EP_PF0_ATU_REGION_CTRL_2_OFF_OUTBOUND_0_EN);

	mscc_fa_vcore_write_reg(priv, 
		PCIE_DM_EP_PF0_ATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_OFF,
		(u32)FDMA_ATU_TARGET_SPACE);
	mscc_fa_vcore_write_reg(priv, 
		PCIE_DM_EP_PF0_ATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0_OFF,
		(u32)(FDMA_ATU_TARGET_SPACE >> 32));
	mscc_fa_vcore_write_reg(priv, 
		PCIE_DM_EP_PF0_ATU_LIMIT_ADDR_OFF_OUTBOUND_0_OFF,
		~0x0);
	mscc_fa_vcore_write_reg(priv, 
		PCIE_DM_EP_PF0_ATU_UPPR_LIMIT_ADDR_OFF_OUTBOUND_0_OFF,
		(u32)(FDMA_ATU_TARGET_SPACE >> 32));
	mscc_fa_vcore_write_reg(priv, 
		PCIE_DM_EP_PF0_ATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0_OFF,
		0);
	mscc_fa_vcore_write_reg(priv, 
		PCIE_DM_EP_PF0_ATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0_OFF,
		0);
}

static int mscc_fa_configure_fdma(struct mscc_fa_fdma *priv, bool use_pcie)
{
	int group = 0;
	int port = 0;
	int channel = 0;
	u32 value = 0;

	pr_debug("%s:%d %s\n", __FILE__, __LINE__, __func__);
	mscc_fa_writel(priv, FDMA_CTRL_OFF, 0);
	wmb();
	mscc_fa_writel(priv, FDMA_CTRL_OFF, FDMA_CTRL_NRESET);

	/* Group: Enable FDMA Injection (mode 2) */
	value = mscc_fa_readl(priv, DEVCPU_QS_INJ_GRP_CFG_R_OFF(group));
	value &= ~DEVCPU_QS_INJ_GRP_CFG_MODE_M;
	value |= DEVCPU_QS_INJ_GRP_CFG_MODE(2);
	mscc_fa_writel(priv, DEVCPU_QS_INJ_GRP_CFG_R_OFF(group), value);

	/* Group: Set gap using IFH (gap 0) */
	mscc_fa_writel(priv, DEVCPU_QS_INJ_CTRL_R_OFF(group),
		DEVCPU_QS_INJ_CTRL_GAP_SIZE(0));

	/* Channel:
	* FDMA_CH_CFG[channel].CH_INJ_PORT is set to 0
	* FDMA_CH_CFG[0-7].CH_DCB_DB_CNT = 1 for number of DCBs
	* FDMA_CH_CFG[0-7].CH_INTR_DB_EOF_ONLY = 1 for intr on EOF only 
	*/
	for (channel = 0; channel < priv->nr_pchans; ++channel) {
		if (use_pcie) {
			mscc_fa_writel(priv, FDMA_CH_CFG_R_OFF(channel), 
				/* Use all data blocks */
				FDMA_CH_CFG_DCB_DB_CNT(FDMA_DCB_MAX_DBS) | 
				FDMA_CH_CFG_INTR_DB_EOF_ONLY |
				FDMA_CH_CFG_MEM);
		}
	}
	/* Group: Enable FDMA Extraction (mode 2) */
	value = mscc_fa_readl(priv, DEVCPU_QS_XTR_GRP_CFG_R_OFF(group));
	value &= ~DEVCPU_QS_XTR_GRP_CFG_MODE_M;
	value |= DEVCPU_QS_XTR_GRP_CFG_MODE(2);
	mscc_fa_writel(priv, DEVCPU_QS_XTR_GRP_CFG_R_OFF(group), value);


	/* This should be in the port handling: fireant_board.c */
	/* Port Format: IFH with no prefix, no preamble, no VSTAX2 aware */
	mscc_fa_writel(priv, ASM_CFG_PORT_R_OFF(port), 
		ASM_CFG_PORT_INJ_FORMAT(1) | 
		ASM_CFG_PORT_NO_PREAMBLE_ENA);

	if (use_pcie) {
		mscc_fa_setup_pcie_atu(priv);
		mscc_fa_access_test(priv);
	}

	return 0;
}

static int mscc_fa_init_region(struct platform_device *pdev,
	struct mscc_fa_fdma *priv, struct resource *res,
	unsigned int top, phys_addr_t vcore_addr)
{
	struct mscc_fa_region *region = 0;

	if (!res->start || !res->end) {
		return -ENOENT;
	}
	region = &priv->regions[GET_REGION_INDEX(top)];
	region->phys_addr = res->start;
	region->size = res->end - res->start + 1;
	region->io_addr = devm_ioremap_resource(&pdev->dev, res);
	region->vcore_addr = vcore_addr;
	pr_debug("%s:%d %s: IO Mapping %u: 0x%llx -> 0x%px .. 0x%px,"
		" vcore: 0x%llx\n",
		__FILE__, __LINE__, __func__, 
		GET_REGION_INDEX(top), 
		region->phys_addr, 
		region->io_addr, 
		region->io_addr + region->size - 1,
		region->vcore_addr);
	return 0;
}

static int mscc_fa_fdma_probe(struct platform_device *pdev)
{
	struct device_node *devnode = pdev->dev.of_node;
	struct mscc_fa_fdma *priv;
	int ret = -ENODEV, nr_channels, i;
	struct resource *res;
	u32 chip_id;

	/* Get Frame DMA info from Device Tree */
	if (!devnode) {
		dev_err(&pdev->dev, "Did not find frame dma device tree node\n");
		return ret;
	}
	ret = of_property_read_u32(devnode, "dma-channels", &nr_channels);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get dma-channels\n");
		goto out_freetreenode;
	}

	dev_info(&pdev->dev, "%d dma-channels\n", nr_channels);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv) + nr_channels *
			sizeof(struct mscc_fa_fdma_channel), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto out_freetreenode;
	}
	priv->nr_pchans = nr_channels;
	platform_set_drvdata(pdev, priv);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);  /* Is this 64? */

	spin_lock_init(&priv->lock);

	/* Use slave mode DMA */
	dma_cap_set(DMA_SLAVE, priv->dma.cap_mask); 
	priv->dma.dev = &pdev->dev;

	tasklet_init(&priv->tasklet, mscc_fa_fdma_tasklet, (unsigned long)priv);

	/* Create the channel list */
	INIT_LIST_HEAD(&priv->dma.channels);
	for (i = 0; i < nr_channels; i++) {
		struct mscc_fa_fdma_channel *fdma_chan = &priv->chans[i];
		fdma_chan->chan.device = &priv->dma;
		fdma_chan->state = DCS_IDLE;
		INIT_LIST_HEAD(&fdma_chan->queued_dcbs);
		list_add_tail(&fdma_chan->chan.device_node, &priv->dma.channels);
	}

	/* Create a pool of consistent memory blocks for hardware descriptors */
	priv->dcb_pool = dmam_pool_create("mscc-fdma-dcb", priv->dma.dev,
		sizeof(struct mscc_fa_fdma_dcb),
		FDMA_BUFFER_ALIGN,
		0);
	if (!priv->dcb_pool) {
		dev_err(&pdev->dev, "Unable to allocate DMA descriptor pool\n");
		ret = -ENOMEM;
		goto out_free;
	}
	INIT_LIST_HEAD(&priv->free_dcbs);
	for (i = 0; i < FDMA_DCB_MAX; ++i) {
		struct mscc_fa_fdma_dcb *dcb;
		dma_addr_t dcb_phys;

		dcb = dma_pool_zalloc(priv->dcb_pool, GFP_ATOMIC, &dcb_phys);
		if (dcb) {
			dcb->phys = dcb_phys;
			dcb->offset = FDMA_ATU_TARGET_SPACE;
			list_add(&dcb->node, &priv->free_dcbs);
		}
	}

	/* Provide DMA Engine device interface */
	priv->dma.dev = &pdev->dev;
	priv->dma.device_alloc_chan_resources = 
		mscc_fa_fdma_alloc_chan_resources;
	priv->dma.device_free_chan_resources = 
		mscc_fa_fdma_free_chan_resources;
	priv->dma.device_prep_slave_sg = mscc_fa_fdma_prep_slave_sg;
	priv->dma.device_tx_status = mscc_fa_fdma_tx_status;
	priv->dma.device_issue_pending = mscc_fa_fdma_issue_pending;
	priv->dma.device_terminate_all = mscc_fa_fdma_terminate;
	priv->dma.device_synchronize = mscc_fa_fdma_sync;
	priv->dma.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	priv->dma.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	priv->dma.directions = BIT(DMA_MEM_TO_MEM);
	priv->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

	/* Register DMA Engine device */
	ret = dma_async_device_register(&priv->dma);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DMA engine device\n");
		goto out_free;
	}

	/* Register DMA controller (uses "dmas" and "dma-names" in DT) */
	ret = of_dma_controller_register(devnode, of_dma_xlate_by_chan_id, priv);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DMA controller\n");
		goto out_unregister;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get IO resource 0\n");
		return -EINVAL;
	}
	mscc_fa_init_region(pdev, priv, res, FIREANT_DEFAULT, VCORE_CSR_SPACE);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get IO resource 1\n");
		return -EINVAL;
	}
	mscc_fa_init_region(pdev, priv, res, FIREANT_AMBA_TOP, AMBA_TOP_SPACE);

	chip_id = mscc_fa_readl(priv, DEVCPU_GCB_CHIP_REGS_ID_OFF);
	pr_debug("%s:%d %s: Fireant: %x\n",
		__FILE__, __LINE__, __func__, chip_id);

	ret = mscc_fa_configure_fdma(priv, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not configure FDMA\n");
		goto out_unregister;
	}

	priv->irq = platform_get_irq(pdev, 0);
	pr_debug("%s:%d %s: IRQ: %d\n", __FILE__, __LINE__, __func__, priv->irq);
	ret = devm_request_irq(&pdev->dev, priv->irq, mscc_fa_fdma_interrupt, 0,
		dev_name(&pdev->dev), priv);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto out_unregister;
	}

	return 0;


out_unregister:
	dma_async_device_unregister(&priv->dma);
out_free:
	kfree(priv);
out_freetreenode:
	of_node_put(devnode);
	return ret;
}

static int mscc_fa_fdma_remove(struct platform_device *pdev)
{
	struct mscc_fa_fdma *priv = platform_get_drvdata(pdev);
	struct mscc_fa_fdma_channel *fdma_chan;
	struct mscc_fa_fdma_dcb *iter;
	int idx, chan;

	pr_debug("%s:%d %s: begin\n", __FILE__, __LINE__, __func__);

	for (chan = 0; chan < priv->nr_pchans; ++chan) {
		fdma_chan = &priv->chans[chan];
		mscc_fa_fdma_free_chan_resources(&fdma_chan->chan);
		mscc_fa_fdma_sync(&fdma_chan->chan);
		mscc_fa_fdma_free_dcb_list(&fdma_chan->chan);
	}

	list_for_each_entry(iter, &priv->free_dcbs, node) {
		pr_debug("%s:%d %s: DCB: 0x%llx: state: %u, cookie: %u\n", 
			__FILE__, __LINE__, __func__, 
			iter->phys,
			iter->state,
			iter->txd.cookie);
		dma_pool_free(priv->dcb_pool, iter, iter->phys);
	}

	/* Free interrupt */
	devm_free_irq(priv->dma.dev, priv->irq, priv);
	for (idx = 0; idx < FIREANT_REGIONS; ++idx) {
		iounmap(priv->regions[idx].io_addr);
	}
	kfree(priv);
	pr_debug("%s:%d %s: end\n", __FILE__, __LINE__, __func__);
	return 0;
}

static const struct of_device_id mscc_fa_fdma_match[] = {
	{ .compatible = "mscc,vsc7568-fdma", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mscc_fa_fdma_match);

static struct platform_driver mscc_fa_fdma_driver = {
	.probe = mscc_fa_fdma_probe,
	.remove = mscc_fa_fdma_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(mscc_fa_fdma_match),
	},
};

static int __init mscc_fa_fdma_init(void)
{
	return platform_driver_register(&mscc_fa_fdma_driver);
}

static void __exit mscc_fa_fdma_exit(void)
{
	platform_driver_unregister(&mscc_fa_fdma_driver);
}

void unload_fa_fdma_driver(void)
{
	pr_debug("%s:%d %s: begin\n", __FILE__, __LINE__, __func__);
	platform_driver_unregister(&mscc_fa_fdma_driver);
	pr_debug("%s:%d %s: end\n", __FILE__, __LINE__, __func__);
}
EXPORT_SYMBOL(unload_fa_fdma_driver);


subsys_initcall(mscc_fa_fdma_init);
module_exit(mscc_fa_fdma_exit);

MODULE_AUTHOR("Steen Hegelund <steen.hegelund@microchip.com>");
MODULE_DESCRIPTION("Microsemi FireAnt FDMA driver");
MODULE_LICENSE("Dual MIT/GPL");
