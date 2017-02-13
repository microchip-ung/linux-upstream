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
 * VCore-III I2C platform data structure
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/i2c.h>
#include <linux/i2c/vcoreiii.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/bug.h>

#include <asm/vcoreiii.h>
#include <asm/vcoreiii-gpio.h>

#define DEBUG_I2C(x...) do { if (unlikely(debug)) printk(KERN_ERR x); } while(0)
#define DEBUG_I2C_L(l, x...) do { if(unlikely(debug >= l)) printk(KERN_ERR x); } while(0)

#define MOD_NAME     "i2c_vcoreiii"

#define VCOREIII_XFER_TIMEOUT       (HZ/4) // 0.25sec
#define VCOREIII_RX_FIFO_FULL_LEVEL 8
#define VCOREIII_TX_FIFO_FULL_LEVEL 8
#define VCOREIII_TX_FIFO_THRESHOLD  6

// Macro for accessing registers - Used for being able to see when we do registers accesses
/* Single-instance macros */
#define VTSS_WRS(data, address) writel(data, address)
#if !defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
/* Only one instance available */
#define VTSS_WR(data, address) writel(data, address)
#define VTSS_RD(address) readl(address)
#else
/* Use the default instance for now */
#define VTSS_WR(data, address) writel(data, address(VTSS_TO_TWI))
#define VTSS_RD(address) readl(address(VTSS_TO_TWI))
#endif

// On JR2, VTSS_F_xxx() macros for single-bit-fields have been
// replaced by VTSS_M_xxx() macros. The VTSS_F_xxx() macros
// all take a parameter.
#if !defined(VTSS_M_TWI_TWI_ENABLE_STATUS_BUSY)
#define VTSS_M_TWI_TWI_ENABLE_STATUS_BUSY VTSS_F_TWI_TWI_ENABLE_STATUS_BUSY
#endif
#if !defined(VTSS_M_TWI_TWI_STAT_TFNF)
#define VTSS_M_TWI_TWI_STAT_TFNF VTSS_F_TWI_TWI_STAT_TFNF
#endif
#if !defined(VTSS_M_TWI_TWI_DATA_CMD_CMD)
#define VTSS_M_TWI_TWI_DATA_CMD_CMD VTSS_F_TWI_TWI_DATA_CMD_CMD
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_STOP_DET)
#define VTSS_M_TWI_TWI_INTR_MASK_M_STOP_DET VTSS_F_TWI_TWI_INTR_MASK_M_STOP_DET
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_RX_OVER)
#define VTSS_M_TWI_TWI_INTR_MASK_M_RX_OVER VTSS_F_TWI_TWI_INTR_MASK_M_RX_OVER
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_RX_UNDER)
#define VTSS_M_TWI_TWI_INTR_MASK_M_RX_UNDER VTSS_F_TWI_TWI_INTR_MASK_M_RX_UNDER
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT)
#define VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT VTSS_F_TWI_TWI_INTR_STAT_TX_ABRT
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_TX_OVER)
#define VTSS_M_TWI_TWI_INTR_STAT_TX_OVER VTSS_F_TWI_TWI_INTR_STAT_TX_OVER
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL)
#define VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL VTSS_F_TWI_TWI_INTR_MASK_M_RX_FULL
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_TX_OVER)
#define VTSS_M_TWI_TWI_INTR_MASK_M_TX_OVER VTSS_F_TWI_TWI_INTR_MASK_M_TX_OVER
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_TX_ABRT)
#define VTSS_M_TWI_TWI_INTR_MASK_M_TX_ABRT VTSS_F_TWI_TWI_INTR_MASK_M_TX_ABRT
#endif
#if !defined(VTSS_M_TWI_TWI_STAT_RFNE)
#define VTSS_M_TWI_TWI_STAT_RFNE VTSS_F_TWI_TWI_STAT_RFNE
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY)
#define VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY VTSS_F_TWI_TWI_INTR_MASK_M_TX_EMPTY
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_STOP_DET)
#define VTSS_M_TWI_TWI_INTR_STAT_STOP_DET VTSS_F_TWI_TWI_INTR_STAT_STOP_DET
#endif
#if !defined(VTSS_M_TWI_TWI_CTRL_ENABLE)
#define VTSS_M_TWI_TWI_CTRL_ENABLE VTSS_F_TWI_TWI_CTRL_ENABLE
#endif
#if !defined(VTSS_M_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE)
#define VTSS_M_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE VTSS_F_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_RX_OVER)
#define VTSS_M_TWI_TWI_INTR_STAT_RX_OVER VTSS_F_TWI_TWI_INTR_STAT_RX_OVER
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_TX_OVER)
#define VTSS_M_TWI_TWI_INTR_STAT_TX_OVER VTSS_F_TWI_TWI_INTR_STAT_TX_OVER
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_RX_FULL)
#define VTSS_M_TWI_TWI_INTR_STAT_RX_FULL VTSS_F_TWI_TWI_INTR_STAT_RX_FULL
#endif
#if !defined(VTSS_M_TWI_TWI_INTR_STAT_TX_EMPTY)
#define VTSS_M_TWI_TWI_INTR_STAT_TX_EMPTY VTSS_F_TWI_TWI_INTR_STAT_TX_EMPTY
#endif
#if !defined(VTSS_M_TWI_TWI_CFG_MASTER_ENA)
#define VTSS_M_TWI_TWI_CFG_MASTER_ENA VTSS_F_TWI_TWI_CFG_MASTER_ENA
#endif
#if !defined(VTSS_M_TWI_TWI_CFG_SLAVE_DIS)
#define VTSS_M_TWI_TWI_CFG_SLAVE_DIS VTSS_F_TWI_TWI_CFG_SLAVE_DIS
#endif
#if !defined(VTSS_M_TWI_TWI_CFG_RESTART_ENA)
#define VTSS_M_TWI_TWI_CFG_RESTART_ENA VTSS_F_TWI_TWI_CFG_RESTART_ENA
#endif


struct vcoreiii_twi_iface {
    struct device      *dev;
    struct i2c_adapter adap;
    int                irq;
    spinlock_t         lock;
    bool               shutdown;

    // Current xfer state;
    struct completion  cmd_complete;
    struct {
        struct i2c_msg     *msg;
#define TX_INCOMPLETE(d) (d->xfer.tx_ix < d->xfer.n_msgs)
#define TX_COMPLETE(d)   (!TX_INCOMPLETE(d))
        int                n_msgs, tx_ix, rx_ix;
        size_t             rx_done, rx_pending, tx_done, tx_byte_ix, rx_byte_ix;
        int                cmd_res;
        int                irqs;
    } xfer;

    // Optional platform hook
    void (*xfer_hook)(struct i2c_adapter *adapter, int start);
};

static int debug;

enum {
    I2C_CMPLT_DONE  = (1 << 0),
    I2C_CMPLT_ABORT = (1 << 1),
    I2C_CMPLT_STOP  = (1 << 2),
};

static void stuff_tx_cont(struct vcoreiii_twi_iface *dev)
{
    /* write to slave */
    while(TX_INCOMPLETE(dev)) {
        // Where are we at?
        struct i2c_msg *txm = dev->xfer.msg + dev->xfer.tx_ix;
        if(dev->xfer.tx_byte_ix < txm->len) {
            // Fifo not full
            u32 lev;
            if((lev = VTSS_RD(VTSS_TWI_TWI_TXFLR)) < VCOREIII_TX_FIFO_FULL_LEVEL) {
                // Read or write fragment?
                if (txm->flags & I2C_M_RD) {
                    if (dev->xfer.rx_pending < VCOREIII_RX_FIFO_FULL_LEVEL) {
                        // Read, issue read cmd, record one pending
                        VTSS_WR(VTSS_M_TWI_TWI_DATA_CMD_CMD, VTSS_TWI_TWI_DATA_CMD);
                        dev->xfer.rx_pending++;
                        DEBUG_I2C_L(3, "Tx <RD> at %d, index %d in frag %d/%d\n",
                                    dev->xfer.tx_done, dev->xfer.tx_byte_ix, dev->xfer.tx_ix, dev->xfer.n_msgs);
                    } else {
                        DEBUG_I2C_L(3, "IRQ[%d]: Max pending RX at %d/%d/%d\n",
                                    dev->xfer.irqs, dev->xfer.tx_done, dev->xfer.rx_done, dev->xfer.rx_pending);
                        break;    // Stop issuing RX commands to avoid RX buffer overrun
                    }
                } else {
                    // Write, just put into FIFO
                    VTSS_WR(txm->buf[dev->xfer.tx_byte_ix], VTSS_TWI_TWI_DATA_CMD);
                    DEBUG_I2C_L(3, "Tx 0x%02x at %d, index %d in frag %d/%d\n",
                                txm->buf[dev->xfer.tx_byte_ix],
                                dev->xfer.tx_done, dev->xfer.tx_byte_ix, dev->xfer.tx_ix, dev->xfer.n_msgs);
                }
                // Done with one byte
                dev->xfer.tx_byte_ix++;
                dev->xfer.tx_done++;
            } else {
                DEBUG_I2C_L(2, "IRQ[%d]: Tx full at %d, index %d in frag %d/%d\n",
                            dev->xfer.irqs, dev->xfer.tx_done, dev->xfer.tx_byte_ix, dev->xfer.tx_ix, dev->xfer.n_msgs);
                break;    // Stop stuffing FIFO
            }
        } else {
            dev->xfer.tx_ix++;    // Done with one fragment
            dev->xfer.tx_byte_ix = 0;    // At start
        }
        // Continue while data
    }
}

static void stuff_tx_start(struct vcoreiii_twi_iface *dev)
{
    // Initial fill level
    VTSS_WR(VCOREIII_TX_FIFO_THRESHOLD, VTSS_TWI_TWI_TX_TL); /* less than 3/4 full, call me */

    // Stuff data
    stuff_tx_cont(dev);

    // Enable IRQs
    VTSS_WR(VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL|
            VTSS_M_TWI_TWI_INTR_MASK_M_RX_OVER|
            VTSS_M_TWI_TWI_INTR_MASK_M_RX_UNDER|
            VTSS_M_TWI_TWI_INTR_MASK_M_STOP_DET|
            VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY|
            VTSS_M_TWI_TWI_INTR_MASK_M_TX_OVER|
            VTSS_M_TWI_TWI_INTR_MASK_M_TX_ABRT,
            VTSS_TWI_TWI_INTR_MASK); // enable tx+rx fifo interrupt
}

static void rx_drain_fifo(struct vcoreiii_twi_iface *dev)
{
    /* Drain Rx FIFO */
    while (VTSS_RD(VTSS_TWI_TWI_STAT) & VTSS_M_TWI_TWI_STAT_RFNE) {
        if (unlikely(dev->xfer.rx_pending == 0)) {
            dev_err(dev->dev, "Rx data, but no pending reads?\n");
            dev->xfer.cmd_res |= I2C_CMPLT_ABORT;
        } else {
            struct i2c_msg *rxm = dev->xfer.msg + dev->xfer.rx_ix;
            BUG_ON(dev->xfer.rx_ix >= dev->xfer.n_msgs);
            if(rxm->flags & I2C_M_RD && dev->xfer.rx_byte_ix < rxm->len) {
                rxm->buf[dev->xfer.rx_byte_ix] = (u8) VTSS_RD(VTSS_TWI_TWI_DATA_CMD);
                dev->xfer.rx_byte_ix++;    // Advance RX buffer offset
                dev->xfer.rx_pending--;    // One less pending
                dev->xfer.rx_done++;       // Record one read
            } else {
                dev->xfer.rx_ix++;    // Done with one fragment (or it was tx)
                dev->xfer.rx_byte_ix = 0;    // At start
            }
        }
    }
}

// Function for restarting the controller. This is needed when the TAR
// register needs to change.
// In : dev - The i2c dev.
static void vcoreiii_enable_controller(struct vcoreiii_twi_iface *dev, u16 addr)
{
    // Disable the I2C controller because TAR register cannot be changed when the controller is enabled.
    VTSS_WR(0x0, VTSS_TWI_TWI_CTRL);

    // Set Target address
    VTSS_WR(addr, VTSS_TWI_TWI_TAR);

    // Enable the I2C controller
    VTSS_WR(VTSS_M_TWI_TWI_CTRL_ENABLE, VTSS_TWI_TWI_CTRL);
}

static void vcoreiii_initial_xfer(struct vcoreiii_twi_iface *dev,
                                  struct i2c_msg *pmsg, int num)
{
    unsigned long flags;
    if (debug) {
        int i;
        for (i = 0; i < num; i++) {
            struct i2c_msg *tmp = pmsg + i;
            DEBUG_I2C("I2C req: frag %2d - %s - %d byte\n", i,
                      tmp->flags & I2C_M_RD ? "rd" : "wr", tmp->len);
        }
    }
    spin_lock_irqsave(&dev->lock, flags);

    memset(&dev->xfer, 0, sizeof(dev->xfer));
    dev->xfer.msg = pmsg;
    dev->xfer.n_msgs = num;
    init_completion(&dev->cmd_complete);

    vcoreiii_enable_controller(dev, pmsg->addr);

    // Stuff data into fifo
    stuff_tx_start(dev);

    spin_unlock_irqrestore(&dev->lock, flags);
}

static void vcoreiii_disable_controller(struct vcoreiii_twi_iface *dev)
{
    /* Disable ints */
    VTSS_WR(0, VTSS_TWI_TWI_INTR_MASK);
    /* Disable the I2C controller */
    VTSS_WR(0x0, VTSS_TWI_TWI_CTRL);
}

static int vcoreiii_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
    struct vcoreiii_twi_iface *dev = i2c_get_adapdata(adap);

    if (dev->shutdown) {
        DEBUG_I2C("VCore-III I2C has been shutdown - return EIO\n");
        return EIO;
    }

    DEBUG_I2C("%s: vcoreiii_xfer - start %d messages:\n", MOD_NAME, num);

    // Platform lock
    if (dev->xfer_hook) {
        dev->xfer_hook(adap, 1);
    }

    /* Safety - TX buffer should be empty */
    BUG_ON(VTSS_RD(VTSS_TWI_TWI_TXFLR) > 0);

    // Start it off
    vcoreiii_initial_xfer(dev, pmsg, num);

    // Wait for it to complete
    num = wait_for_completion_interruptible_timeout(&dev->cmd_complete, VCOREIII_XFER_TIMEOUT);

    vcoreiii_disable_controller(dev);

    DEBUG_I2C("Xfer(0x%0x) complete: irqs: %d, xmit %d, read %d, cmd_res: 0x%x\n",
              pmsg->addr, dev->xfer.irqs, dev->xfer.tx_done, dev->xfer.rx_done, dev->xfer.cmd_res);

    if (likely(num > 0)) {
        if (dev->xfer.cmd_res & I2C_CMPLT_DONE) {
            /* no error */
            num = dev->xfer.tx_done;
        } else {
            // Transfer failed
            num = -EIO;
        }
    } else if (num == 0) {
        DEBUG_I2C("controller timed out\n");
        num = -ETIMEDOUT;
    } else {
        DEBUG_I2C("Interrupted\n");
        // num already set
    }

    // Platform unlock
    if (dev->xfer_hook) {
        dev->xfer_hook(adap, 0);
    }

    return num;
}

static unsigned int vcoreiii_func(struct i2c_adapter *adapter)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static irqreturn_t vcoreiii_twi_interrupt_entry(int irq, void *dev_id)
{
    struct vcoreiii_twi_iface *dev = dev_id;
    u32 ostatus, status, irq_mask;
    int count = 0;

    dev->xfer.irqs++;

    while((ostatus = status = VTSS_RD(VTSS_TWI_TWI_INTR_STAT))) { // figure out what interrupt we got

        if (count++ == 100) {
            dev_err(dev->dev, "Too much work in one IRQ - irqstat 0x%0x, status 0x%0x\n", ostatus, VTSS_RD(VTSS_TWI_TWI_STAT));
            break;
        }

	if(status & VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT) {
            status &= ~VTSS_M_TWI_TWI_INTR_STAT_TX_ABRT; /* clear tx_abrt */
            DEBUG_I2C("IRQ[%d]: 0x%x - TX abrt src 0x%x\n", dev->xfer.irqs, ostatus, VTSS_RD(VTSS_TWI_TWI_TX_ABRT_SOURCE));
            dev->xfer.cmd_res |= I2C_CMPLT_ABORT;
            break;
        }

        // These are always ignored if present
        status &= ~(VTSS_M_TWI_TWI_INTR_STAT_TX_EMPTY|VTSS_M_TWI_TWI_INTR_MASK_M_RX_FULL);

        // Rx data there?
        rx_drain_fifo(dev);

        // Tx to stuff?
        stuff_tx_cont(dev);

        // Quiten TX IRQ's
        irq_mask = VTSS_RD(VTSS_TWI_TWI_INTR_MASK);
        if (TX_INCOMPLETE(dev) && dev->xfer.rx_pending < VCOREIII_RX_FIFO_FULL_LEVEL) {
            // We can allow more TX irq's
            irq_mask |= VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY;
        } else {
            // Shut off TX irq for now; no more tx data, or rx requests maxed out
            irq_mask &= ~VTSS_M_TWI_TWI_INTR_MASK_M_TX_EMPTY;
        }
        VTSS_WR(irq_mask, VTSS_TWI_TWI_INTR_MASK); // On/off TX IRQ

        // Stop seen?
	if(status & VTSS_M_TWI_TWI_INTR_STAT_STOP_DET) {
            u32 rx_lev, tx_lev;
            status &= ~VTSS_M_TWI_TWI_INTR_STAT_STOP_DET; /* clear stop_det */
            (void) VTSS_RD(VTSS_TWI_TWI_CLR_STOP_DET);
            DEBUG_I2C_L(2, "Stop detect: RX %d/%d, offset %d, pending %d, have %d\n",
                        dev->xfer.n_msgs, dev->xfer.rx_ix, dev->xfer.rx_byte_ix,
                        dev->xfer.rx_pending, dev->xfer.rx_done);
            // See if tx/rx in FIFO empty:
            if ((tx_lev = VTSS_RD(VTSS_TWI_TWI_TXFLR)) == 0 &&
                (rx_lev = VTSS_RD(VTSS_TWI_TWI_RXFLR)) == 0) {
                if(TX_COMPLETE(dev) && dev->xfer.rx_pending == 0) {
                    DEBUG_I2C("IRQ[%d]: Stop at FIFO empty, Tx complete and no pending reads at %d/%d\n",
                              dev->xfer.irqs, dev->xfer.tx_done, dev->xfer.rx_done);
                    dev->xfer.cmd_res |= I2C_CMPLT_DONE;
                } else {
                    if (dev->xfer.rx_pending != 0) {
                        DEBUG_I2C("IRQ[%d]: Rx stop, we are incomplete at tx %d, rx %d, pending %d\n",
                                  dev->xfer.irqs, dev->xfer.tx_done, dev->xfer.rx_done, dev->xfer.rx_pending);
                        dev->xfer.cmd_res |= I2C_CMPLT_STOP;
                    } else {
                        dev_err(dev->dev, "Stop at incomplete Tx, but fifo empty? status = 0x%08x\n",
                                VTSS_RD(VTSS_TWI_TWI_STAT));
                    }
                }
            } else {
                DEBUG_I2C("IRQ[%d]: Stop with FIFO's at tx %d / rx %d, status = 0x%08x\n",
                          dev->xfer.irqs, tx_lev, rx_lev, VTSS_RD(VTSS_TWI_TWI_STAT));
            }
        }

        // What else
        if (unlikely(status)) {
            dev_err(dev->dev, "Unexpected status 0x%0x (0x%0x)\n", status, ostatus);
        }
    }

    if (dev->xfer.cmd_res) {
        vcoreiii_disable_controller(dev);
        complete(&dev->cmd_complete);
    }

    return count ? IRQ_HANDLED : IRQ_NONE;
}

static struct i2c_algorithm i2c_vcoreiii_algo = {
    .master_xfer    = vcoreiii_xfer,
    .functionality  = vcoreiii_func,
};

static int i2c_vcoreiii_hwinit(const struct vcoreiii_i2c_platform_data *pdata)
{
    unsigned long clk_freq = VCOREIII_AHB_CLOCK, reg_val;

    reg_val = (5 * clk_freq / 1000000) - 8;  // datasheet 6.17.1.5
    reg_val *= 2;                            // Lower by 50% due to lack of stretching
    VTSS_WR(reg_val, VTSS_TWI_TWI_SS_SCL_HCNT);

    reg_val = (5 * clk_freq / 1000000) - 1;  // datasheet 6.17.1.6
    reg_val *= 2;                            // Lower by 50% due to lack of stretching
    VTSS_WR(reg_val, VTSS_TWI_TWI_SS_SCL_LCNT);

    reg_val = VTSS_F_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_CNT_RELOAD((unsigned int)(0.3 * clk_freq / 1000000) - 1) | VTSS_M_ICPU_CFG_TWI_DELAY_TWI_CONFIG_TWI_DELAY_ENABLE;  // datasheet 6.17
    VTSS_WRS(reg_val, VTSS_ICPU_CFG_TWI_DELAY_TWI_CONFIG);

    reg_val = (1.1 * clk_freq / 1000000) - 1;  // datasheet 6.17.1.7
    VTSS_WR(reg_val, VTSS_TWI_TWI_FS_SCL_HCNT);

    reg_val = (1.4 * clk_freq / 1000000) - 1;  // datasheet 6.17.1.8
    VTSS_WR(reg_val, VTSS_TWI_TWI_FS_SCL_LCNT);

    reg_val =
            VTSS_M_TWI_TWI_CFG_MASTER_ENA |
            VTSS_F_TWI_TWI_CFG_SPEED(pdata->fast_mode ? 2 : 1) | /* 400 or 100 kbit/s */
            VTSS_M_TWI_TWI_CFG_RESTART_ENA |
            VTSS_M_TWI_TWI_CFG_SLAVE_DIS;
    VTSS_WR(reg_val, VTSS_TWI_TWI_CFG);

    reg_val = (0.25 * clk_freq / 1000000);  // datasheet 6.17.1.30
    VTSS_WR(reg_val, VTSS_TWI_TWI_SDA_SETUP);

    VTSS_WR(0, VTSS_TWI_TWI_RX_TL); /* (n+1) => one byte of data */
    VTSS_WR(0x0, VTSS_TWI_TWI_INTR_MASK); // mask all until we're ready
    VTSS_WR(VTSS_M_TWI_TWI_CTRL_ENABLE, VTSS_TWI_TWI_CTRL);

    return 0;
}

static struct vcoreiii_twi_iface *reboot_iface;
static int vcoreiii_i2c_notify_reboot(struct notifier_block *this,
                                      unsigned long code,
                                      void *x)
{
    if (reboot_iface->xfer.cmd_res == 0) {
        printk(KERN_ERR "VcoreIII I2C: Disabling with active transfer pending\n");
    }

    vcoreiii_disable_controller(reboot_iface);
    reboot_iface->shutdown = true;

    return NOTIFY_DONE;
}

static struct notifier_block vcoreiii_i2c_notifier = {
	.notifier_call	= vcoreiii_i2c_notify_reboot,
	.priority	= 0,
};

static struct vcoreiii_i2c_platform_data i2c_data_default = {
    .fast_mode = 0,             /* 100 kbit/s */
};

static int i2c_vcoreiii_probe(struct platform_device *pdev)
{
    struct vcoreiii_i2c_platform_data *pdata = pdev->dev.platform_data;
    struct vcoreiii_twi_iface *iface;
    struct i2c_adapter *adapter;
    int rc=1;

    DEBUG_I2C_L(2, "%s: i2c_vcoreiii_probe, pdata %p\n", MOD_NAME, pdata);
    if(!pdata)
        pdata = &i2c_data_default;

#if defined(CONFIG_VTSS_VCOREIII_JAGUAR)
    vcoreiii_gpio_set_alternate(14, 1); /* TWI_SCL */
    vcoreiii_gpio_set_alternate(15, 1); /* TWI_SDA */
#elif defined(CONFIG_VTSS_VCOREIII_LUTON26)
    vcoreiii_gpio_set_alternate(5, 1); /* TWI_SCL */
    vcoreiii_gpio_set_alternate(6, 1); /* TWI_SDA */
#elif defined(CONFIG_VTSS_VCOREIII_SERVALT)
    vcoreiii_gpio_set_alternate(24, 1); /* TWI_SDA */
    vcoreiii_gpio_set_alternate(25, 1); /* TWI_SCL */
#elif defined(CONFIG_VTSS_VCOREIII_JAGUAR2_FAMILY)
    vcoreiii_gpio_set_alternate(14, 1); /* TWI_SCL */
    vcoreiii_gpio_set_alternate(15, 1); /* TWI_SDA */
#elif defined(CONFIG_VTSS_VCOREIII_SERVAL1)
    vcoreiii_gpio_set_alternate(11, 1); /* TWI_SCL */
    vcoreiii_gpio_set_alternate(7, 1); /* TWI_SDA */
#else
#error Unsupported platform!
#endif

    i2c_vcoreiii_hwinit(pdata);

    reboot_iface = iface = kzalloc(sizeof(struct vcoreiii_twi_iface), GFP_KERNEL);
    if(iface == NULL) {
        dev_err(&pdev->dev, "can't allocate interface\n");
        rc = -ENOMEM;
        goto fail0;
    }

    iface->dev = get_device(&pdev->dev);
    iface->irq = TWI_IRQ;
    iface->xfer_hook = pdata->xfer_hook;    // Optional platform hook
    spin_lock_init(&iface->lock);
    platform_set_drvdata(pdev, iface);

    adapter = &iface->adap;
    i2c_set_adapdata(adapter, iface);
    adapter->owner = THIS_MODULE;
    adapter->class = 0;    // No probing, please!
    snprintf(adapter->name, sizeof(adapter->name), MOD_NAME);
    adapter->algo = &i2c_vcoreiii_algo;
    adapter->dev.parent = &pdev->dev;

    rc = request_irq(iface->irq, vcoreiii_twi_interrupt_entry, 0, pdev->name, iface);
    if (rc) {
	dev_err(&pdev->dev, "Can't get IRQ %d !\n", iface->irq);
	rc = -ENODEV;
	goto fail1;
    }

    rc = i2c_add_numbered_adapter(adapter);
    if(rc) {
        dev_err(&pdev->dev, "Adapter %s registration failed\n", adapter->name);
        goto fail2;
    }

    register_reboot_notifier(&vcoreiii_i2c_notifier);

    dev_info(&pdev->dev, "i2c bus driver on IRQ %d\n", iface->irq);
    return 0;

fail2:
    free_irq(iface->irq, iface);
fail1:
    platform_set_drvdata(pdev, NULL);
    kfree(adapter);
fail0:
    return 0;
}

static int i2c_vcoreiii_remove(struct platform_device *pdev)
{
    struct vcoreiii_twi_iface *iface = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "Driver remove (releasing IRQ %d)\n", iface->irq);

    unregister_reboot_notifier(&vcoreiii_i2c_notifier);

    platform_set_drvdata(pdev, NULL);

    i2c_del_adapter(&(iface->adap));
    free_irq(iface->irq, iface);
    kfree(iface);

    return 0;
}

static struct platform_driver i2c_vcoreiii_driver = {
	.probe		= i2c_vcoreiii_probe,
	.remove		= i2c_vcoreiii_remove,
	.driver		= {
		.name	= MOD_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init i2c_vcoreiii_init(void)
{
    return platform_driver_register(&i2c_vcoreiii_driver);
}
subsys_initcall(i2c_vcoreiii_init);

static void __exit i2c_vcoreiii_exit(void)
{
    platform_driver_unregister(&i2c_vcoreiii_driver);
}
module_exit(i2c_vcoreiii_exit);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug");

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_vcoreiii");
MODULE_AUTHOR("Lars Povlsen <lpovlsen at vitesse.com>");
MODULE_DESCRIPTION("Vitesse VCore-III I2C bus adapter");
MODULE_LICENSE("GPL");
