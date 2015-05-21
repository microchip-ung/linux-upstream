/*

 Vitesse Switch Software.

 Copyright (c) 2002-2015 Vitesse Semiconductor Corporation "Vitesse". All
 Rights Reserved.

 Unpublished rights reserved under the copyright laws of the United States of
 America, other countries and international treaties. Permission to use, copy,
 store and modify, the software and its source code is granted. Permission to
 integrate into other products, disclose, transmit and distribute the software
 in an absolute machine readable format (e.g. HEX file) is also granted.  The
 source code of the software may not be disclosed, transmitted or distributed
 without the written permission of Vitesse. The software and its source code
 may only be used in products utilizing the Vitesse switch products.

 This copyright notice must appear in any copy, modification, disclosure,
 transmission or distribution of the software. Vitesse retains all ownership,
 copyright, trade secret and proprietary rights in the software.

 THIS SOFTWARE HAS BEEN PROVIDED "AS IS," WITHOUT EXPRESS OR IMPLIED WARRANTY
 INCLUDING, WITHOUT LIMITATION, IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR USE AND NON-INFRINGEMENT.

*/

#define UFDMA_TRACE_LAYER VTSS_UFDMA_TRACE_LAYER_AIL

#include "ufdma.h"
#ifdef __KERNEL__
#include <linux/string.h> /* For memset() */
#else
#include <string.h>       /* For memset() */
#endif

/**
 * If CIL function exists, result of this macro is the result
 * of invoking the CIL function, otherwise it is UFDMA_RC_NO_SUCH_CIL_FUNC.
 */
#define UFDMA_CIL_FUNC(state, cil_func, ...) ((state)->cil.cil_func == NULL ? UFDMA_RC_NO_SUCH_CIL_FUNC : state->cil.cil_func(state, ##__VA_ARGS__))

/**
 * If CIL function exists, returns if CIL function fails, otherwise continues.
 * If CIL function doesn't exist, UFDMA_RC_NO_SUCH_CIL_FUNC is returned.
 * '##' means: Delete preceding comma if no args.
 */
#define UFDMA_CIL_FUNC_RC(state, cil_func, ...) UFDMA_RC(UFDMA_CIL_FUNC(state, cil_func, ##__VA_ARGS__))

/******************************************************************************/
//
// Private functions
//
/******************************************************************************/

/**
 * AIL_self_chk_do()
 */
static int AIL_self_chk_do(vtss_ufdma_platform_driver_t *self, ufdma_state_t **state)
{
    if (self == NULL) {
        return UFDMA_RC_SELF;
    }

    if (self->state == NULL) {
        return UFDMA_RC_STATE;
    }

    // No matter which pointer the network driver comes with, align it to an 8-byte address,
    // so that we can store a structure on top of it.
    // We made sure to tell the network driver to allocate enough memory for this
    // alignment in props.ufdma_state_size_bytes.
    *state = (ufdma_state_t *)UFDMA_ALIGNED_SIZE(self->state, 8);

    return UFDMA_RC_OK;
}

/**
 * AIL_self_chk_no_init()
 */
static int AIL_self_chk_no_init(vtss_ufdma_platform_driver_t *self, ufdma_state_t **state)
{
    UFDMA_RC(AIL_self_chk_do(self, state));

    if ((*state)->initialized) {
        return UFDMA_RC_ALREADY_INITIALIZED;
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_self_chk()
 */
static int AIL_self_chk(vtss_ufdma_platform_driver_t *self, ufdma_state_t **state)
{
    UFDMA_RC(AIL_self_chk_do(self, state));

    if (!(*state)->initialized) {
        return UFDMA_RC_NOT_INITIALIZED;
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_buf_dscr_chk()
 */
static int AIL_buf_dscr_chk(ufdma_state_t *state, vtss_ufdma_buf_dscr_t *buf_dscr, BOOL rx)
{
    u32 min, max;

    if (buf_dscr == NULL || buf_dscr->buf == NULL) {
        return UFDMA_RC_ARG;
    }

    if (rx) {
        min = state->cil.rx_buf_size_bytes_min;
        max = state->cil.rx_buf_size_bytes_max;
    } else {
        min = state->self->props.tx_ifh_size_bytes + 2 * 6 /* DMAC + SMAC */ + 2 /* EtherType */ + 4 /* FCS */;
        max = state->cil.tx_buf_size_bytes_max;
    }

    if (buf_dscr->buf_size_bytes < min || buf_dscr->buf_size_bytes > max) {
        return UFDMA_RC_BUF_SIZE;
    }

    if (buf_dscr->buf_state == NULL) {
        return UFDMA_RC_BUF_STATE;
    }

    if (rx && ((u32)buf_dscr->buf & 0x3) != 0) {
        return UFDMA_RC_RX_BUF_ALIGNMENT;
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_debug_print_dcb_list()
 */
static void AIL_debug_print_dcb_list(ufdma_state_t *state, BOOL full, int (*pr)(const char *fmt, ...), ufdma_dcb_t *dcb, char *name)
{
    u32  cnt = 1;

    pr("%s:%s\n", name, dcb ? "" : " NULL");
    while (dcb) {
        void *llp;

        // Print all of them or the first four and the last.
        if (full || cnt <= 4 || dcb->next == NULL) {
            pr(" %3u: 0x%08x[&dscr=%p, dscr.buf=%p (%u), dscr.buf_state=%p, hw->next=", cnt, dcb, &dcb->buf_dscr, dcb->buf_dscr.buf, dcb->buf_dscr.buf_size_bytes, dcb->buf_dscr.buf_state);
            llp = state->cil.hw_dcb_next(state, dcb);

            if (llp) {
                pr("%p]\n", llp);
            } else {
                pr("NULL]\n");
            }
        } else if (cnt == 5) {
            pr(" ...\n");
        }

        dcb = dcb->next;
        cnt++;
    }

    pr("\n");
}

/**
 * AIL_debug_print_do()
 */
static int AIL_debug_print_do(ufdma_state_t *state, vtss_ufdma_debug_info_t *info, int (*pr)(const char *fmt, ...))
{
    vtss_ufdma_throttle_conf_t *throttle_conf  = &state->throttle_conf;
    ufdma_throttle_state_t     *throttle_state = &state->throttle_state;
    u32                        rx_qu;
    BOOL                       pr_full = (info->full != 0);
    BOOL                       pr_rx   = (info->group == VTSS_UFDMA_DEBUG_GROUP_ALL || info->group == VTSS_UFDMA_DEBUG_GROUP_RX);
    BOOL                       pr_tx   = (info->group == VTSS_UFDMA_DEBUG_GROUP_ALL || info->group == VTSS_UFDMA_DEBUG_GROUP_TX);

    if (pr_rx) {
        pr("Rx\n");
        pr("--\n");
        pr("Poll calls:      %12u\n",    state->stati.rx_poll_calls);
        pr("Buf Add calls:   %12u\n",    state->stati.rx_buf_add_calls);
        pr("Callback calls:  %12u\n",    state->stati.rx_callback_calls);
        pr("Callback bytes:  %12llu\n",  state->stati.rx_callback_bytes);
        pr("Multi-DCB drops: %12u\n",    state->stati.rx_multi_dcb_drops);
        pr("Oversize drops:  %12u\n",    state->stati.rx_oversize_drops);
        pr("Abort drops:     %12u\n",    state->stati.rx_abort_drops);
        pr("Pruned drops:    %12u\n",    state->stati.rx_pruned_drops);
        pr("Suspended drops: %12u\n",    state->stati.rx_suspended_drops);
        pr("CIL drops:       %12u\n\n",  state->stati.rx_cil_drops);

        pr("Qu Frames       Bytes\n");
        pr("-- ------------ ------------\n");
        for (rx_qu = 0; rx_qu < ARRSZ(state->stati.rx_frms); rx_qu++) {
            pr("%2u %12u %12llu\n", rx_qu, state->stati.rx_frms[rx_qu], state->stati.rx_bytes[rx_qu]);
        }

        // Print throttle statistics.
        pr("\nThrottle ticks: %llu\n", throttle_state->tick_cnt);
        pr("Qu Frame Limit/Tick Byte Limit/Tick Max Frames/Tick Max Bytes/Tick # Suspends Suspend Ticks Left\n");
        pr("-- ---------------- --------------- --------------- -------------- ---------- ------------------\n");
        for (rx_qu = 0; rx_qu < ARRSZ(throttle_conf->frm_limit_per_tick); rx_qu++) {
            u32 frm_limit  = throttle_conf->frm_limit_per_tick[rx_qu];
            u32 byte_limit = throttle_conf->byte_limit_per_tick[rx_qu];
            pr("%2u %16u %15u %15u %14u %10u %18u\n", rx_qu, frm_limit, byte_limit, throttle_state->statistics_max_frames_per_tick[rx_qu], throttle_state->statistics_max_bytes_per_tick[rx_qu], throttle_state->suspend_cnt[rx_qu], throttle_state->ticks_left[rx_qu]);
        }

        pr("\nRx Pointers\n");
        pr("-----------\n");
        AIL_debug_print_dcb_list(state, pr_full, pr, state->rx_head_sw, "rx_head_sw");
        AIL_debug_print_dcb_list(state, pr_full, pr, state->rx_head_hw, "rx_head_hw");

    }

    if (pr_tx) {
        pr("\nTx\n");
        pr("--\n");
        pr("Poll calls:      %12u\n",   state->stati.tx_poll_calls);
        pr("Tx calls:        %12u\n",   state->stati.tx_calls);
        pr("Tx bytes:        %12llu\n", state->stati.tx_bytes);
        pr("Callback calls:  %12u\n",   state->stati.tx_done_callback_calls);

        pr("\nTx Pointers\n");
        pr("-----------\n");
        AIL_debug_print_dcb_list(state, pr_full, pr, state->tx_head_sw, "tx_head_sw");
        AIL_debug_print_dcb_list(state, pr_full, pr, state->tx_tail_sw, "tx_tail_sw");
        AIL_debug_print_dcb_list(state, pr_full, pr, state->tx_head_hw, "tx_head_hw");
    }

    pr("\nRx/Tx\n");
    pr("-----\n");
    pr("Poll calls:      %12u\n", state->stati.poll_calls);

    return UFDMA_RC_OK;
}

/**
 * AIL_reorder_barrier()
 */
static int AIL_reorder_barrier(ufdma_state_t *state)
{
    // Empty on purpose.
    return UFDMA_RC_OK;
}

/**
 * AIL_reg_rd()
 */
static u32 AIL_reg_rd(ufdma_state_t *state, u32 addr)
{
    u32 val = state->callout.reg_read(0, addr);

    UFDMA_D("R: addr = %u, val = 0x%08x", addr, val);
    return val;
}

/**
 * AIL_reg_wr()
 */
static void AIL_reg_wr(ufdma_state_t *state, u32 addr, u32 val)
{
    UFDMA_D("W: addr = %u, val = 0x%08x", addr, val);
    state->callout.reg_write(0, addr, val);
}

/**
 * UFDMA_dcb_prepare()
 * Takes the area allocated by the user and stores uFDMA-private data.
 */
static ufdma_dcb_t *UFDMA_dcb_prepare(vtss_ufdma_platform_driver_t *self, vtss_ufdma_buf_dscr_t *buf_dscr)
{
    ufdma_dcb_t *dcb;

    // Reset the area allocated by the user to hold the S/W and H/W DCB.
    memset(buf_dscr->buf_state, 0, self->props.buf_state_size_bytes);

    // Fit in a ufdma_dcb_t in the user-allocated buffer state.
    // It must be cache aligned, because the very first field of
    // it is the H/W DCB, which requires this.
    // #buf_state_size_bytes accommodates for this cache-alignment.
    dcb = (ufdma_dcb_t *)UFDMA_CACHE_ALIGNED_SIZE(buf_dscr->buf_state);

    // Now get a copy of the user's buffer description into ourselves.
    dcb->buf_dscr = *buf_dscr;

    return dcb;
}

/**
 * AIL_cpu_to_bus()
 *
 * When filling in or extracting info from DCBs,
 * we need to do it to or from the native bus's endianness,
 * which is little endian. Whether we have to swap
 * the bytes or not depends on the CPU's endianness.
 */
static int AIL_cpu_to_bus(ufdma_state_t *state, u32 cpu, u32 *bus)
{
    if (state->callout.big_endian) {
        register u32 v1 = cpu;
        v1 = ((v1 >> 24) & 0x000000FF) | ((v1 >> 8) & 0x0000FF00) | ((v1 << 8) & 0x00FF0000) | ((v1 << 24) & 0xFF000000);
        *bus = v1;
    } else {
        *bus = cpu;
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_bus_to_cpu()
 */
static int AIL_bus_to_cpu(ufdma_state_t *state, u32 bus, u32 *cpu)
{
    return AIL_cpu_to_bus(state, bus, cpu);
}

/**
 * AIL_rx_qu_suspend_set_all()
 */
static int AIL_rx_qu_suspend_set_all(ufdma_state_t *state, BOOL suspend)
{
    unsigned int rx_qu;

    for (rx_qu = 0; rx_qu < ARRSZ(state->throttle_conf.frm_limit_per_tick); rx_qu++) {
        UFDMA_CIL_FUNC_RC(state, rx_qu_suspend_set, rx_qu, FALSE);
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_rx_restart()
 */
static int AIL_rx_restart(ufdma_state_t *state)
{
    BOOL restarted;

    // For the sake of Serval, we cannot test whether state->rx_head_hw is NULL
    // here, because it may be non-NULL at the same time as the H/W indicates
    // that the channel's DCBs have depleted. Instead, we just call the
    // CIL function and checks its return value, and connect the new head
    // to the end of cur_head.
    if (state->rx_head_sw != NULL) {
        // Potentially restart the channel. If this function sets #restarted to TRUE,
        // the H/W has indeed been restarted.
        UFDMA_CIL_FUNC_RC(state, rx_start, state->rx_head_sw, &restarted);

        if (restarted) {
            UFDMA_IG(RX, "Restarted with %p. rx_head_hw = %p", state->rx_head_sw, state->rx_head_hw);

            // Update S/W pointers.
            if (state->rx_head_hw) {
                // Should only occur on Serval.
                // Connect rx_head_sw to the end of the rx_head_hw (which may or may not be NULL).
                // We only need to connect the two lists S/W-wise - not H/W-wise.
                ufdma_dcb_t *temp = state->rx_head_hw;
                while (temp->next) {
                    temp = temp->next;
                }

                temp->next = state->rx_head_sw;
            } else {
                state->rx_head_hw = state->rx_head_sw;
            }

            state->rx_head_sw = NULL;
        }
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_rx_buf_add_do()
 */
static int AIL_rx_buf_add_do(ufdma_state_t *state, ufdma_dcb_t *dcb)
{
    u32 b = state->cil.rx_burst_size_bytes, buf_size_bytes_aligned;

    // Link the DCB into the start of the current S/W list.
    dcb->next = state->rx_head_sw;
    state->rx_head_sw = dcb;

    // Align the length that we configure into the FDMA to be the closest HIGHER
    // multiple of the platform's burst size.
    // We can safely give it more than what the buffer specifies, because we
    // know that the user has called rx_buf_alloc_size_get() to get the size
    // he must allocate to hold a buffer of buf_size_bytes bytes.
    buf_size_bytes_aligned = b * ((dcb->buf_dscr.buf_size_bytes + b - 1) / b);

    // Ask CIL to initialize the H/W area and link it to the current head.
    UFDMA_CIL_FUNC_RC(state, rx_dcb_init, dcb, buf_size_bytes_aligned);

    // If the user has modified the buffer, we need to invalidate the cache
    // before the FDMA has actually written data to the buffer because
    // part of the act of invalidating the cache is to drain the write buffer,
    // and if we waited with invalidating the cache until after the FDMA has
    // filled data into it, the FDMA-filled data would be overwritten with
    // the stale data in the write buffer.
    DCACHE_INVALIDATE(dcb->buf_dscr.buf, dcb->buf_dscr.buf_size_bytes);

    return UFDMA_RC_OK;
}

/**
 * AIL_rx_buf_recycle()
 * Move head of Rx buffer in H/W to head of
 * Rx buffer of S/W, while initializing it.
 */
static int AIL_rx_buf_recycle(ufdma_state_t *state)
{
    ufdma_dcb_t *dcb_next = state->rx_head_hw->next;

    // Link the DCB into the start of the current S/W list
    // and ask CIL to initialize the H/W area and link it to the current head.
    UFDMA_RC(AIL_rx_buf_add_do(state, state->rx_head_hw));

    // And remove it from the head of the H/W list
    state->rx_head_hw = dcb_next;

    return UFDMA_RC_OK;
}

/**
 * AIL_rx_buf_recycle_all()
 * Move all Rx buffers currently given to H/W
 * back to S/W while initializing them.
 * Retart Rx afterwards.
 */
static int AIL_rx_buf_recycle_all(ufdma_state_t *state)
{
    while (state->rx_head_hw) {
        UFDMA_RC(AIL_rx_buf_recycle(state));
    }

    // Check if we need to restart the channel.
    return AIL_rx_restart(state);
}

/**
 * AIL_rx_throttle_suspend_check()
 */
static int AIL_rx_throttle_suspend_check(ufdma_state_t *state, u32 rx_qu, u32 bytes, BOOL *drop)
{
    vtss_ufdma_throttle_conf_t *throttle_conf  = &state->throttle_conf;
    ufdma_throttle_state_t     *throttle_state = &state->throttle_state;
    BOOL                       suspend_due_to_frm_cnt, suspend_due_to_byte_cnt;

    throttle_state->frm_cnt[rx_qu]++;
    throttle_state->byte_cnt[rx_qu] += bytes;

    suspend_due_to_byte_cnt = throttle_conf->byte_limit_per_tick[rx_qu] > 0 && throttle_state->byte_cnt[rx_qu] > throttle_conf->byte_limit_per_tick[rx_qu];
    suspend_due_to_frm_cnt  = throttle_conf->frm_limit_per_tick[rx_qu]  > 0 && throttle_state->frm_cnt[rx_qu]  > throttle_conf->frm_limit_per_tick[rx_qu];

    if ((suspend_due_to_frm_cnt || suspend_due_to_byte_cnt) && throttle_state->tick_cnt > 1) {
        *drop = TRUE;

        if (throttle_state->ticks_left[rx_qu] == 0) {
            // Not already suspended. Suspend it.
            UFDMA_CIL_FUNC_RC(state, rx_qu_suspend_set, rx_qu, TRUE);

            // And let vtss_ufdma_platform_driver_t::throttle_tick() re-enable the queue once
            // the suspension period elapses.
            // We add one to the requested suspend_tick_cnt because we are anywhere in between
            // two throttle ticks, and the suspension period starts upon the next throttle tick.
            throttle_state->ticks_left[rx_qu] = throttle_conf->suspend_tick_cnt[rx_qu] + 1;

            // Count the event.
            throttle_state->suspend_cnt[rx_qu]++;
        }
    } else {
        *drop = FALSE;
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_rx_qu_from_mask()
 *
 * The Rx queue mask is a mask of Rx queues where each bit implies that
 * the frame was subject to CPU forwarding onto the specific Rx queue.
 * The most significant of these bits indicates the Rx queue that the
 * frame was actually received on.
 * This function finds this queue number (0-based) from the mask.
 */
static u32 AIL_rx_qu_from_mask(u32 rx_qu_mask)
{
    int rx_qu; // Must be an integer to be able to terminate this loop if called with 0.

    for (rx_qu = 8 /* Rx Queue Cnt */ - 1; rx_qu >= 0; rx_qu--) {
        if (rx_qu_mask & (1 << rx_qu)) {
            break;
        }
    }

    return (u32)rx_qu;
}

/**
 * AIL_rx_frm()
 */
static int AIL_rx_frm(ufdma_state_t *state, u32 chip_no)
{
    vtss_ufdma_buf_dscr_t buf_dscr;
    ufdma_hw_dcb_status_t dcb_status;
    ufdma_dcb_t           *new_head;
    u32                   rx_qu_mask, rx_qu;
    BOOL                  drop;

    // Iterate over all DCBs and stop when we reach one that has not been filled in by H/W.
    while (state->rx_head_hw) {

        // Get this DCB's status
        UFDMA_CIL_FUNC_RC(state, dcb_status_decode, state->rx_head_hw, &dcb_status, TRUE /* is_rx = TRUE */);
        UFDMA_DG(RX, "fragment_size_bytes = %u, sof = %u, eof = %u, aborted = %u, pruned = %u",
                 dcb_status.fragment_size_bytes,
                 dcb_status.sof,
                 dcb_status.eof,
                 dcb_status.aborted,
                 dcb_status.pruned);

        if (dcb_status.fragment_size_bytes == 0) {
            // DCB not filled in by H/W yet.
            break;
        }

        if (dcb_status.aborted) {
            UFDMA_EG(RX, "Frame was aborted. Recycling DCB");
            state->stati.rx_abort_drops++;
            AIL_rx_buf_recycle(state);
            continue;
        }

        if (dcb_status.pruned) {
            UFDMA_EG(RX, "Frame was pruned. Recycling DCB");
            state->stati.rx_pruned_drops++;
            AIL_rx_buf_recycle(state);
            continue;
        }

        if (dcb_status.sof) {
            // This is a start-of-frame DCB.
            if (!dcb_status.eof) {
                // We received a multi-DCB frame. We don't support this in this driver,
                // so count it and recycle the DCB.
                state->stati.rx_multi_dcb_drops++;
                AIL_rx_buf_recycle(state);
                continue;
            }
        } else {
            // This is not a start-of-frame DCB.
            // No matter what, recycle it, because we don't support
            // frames that span multiple DCBs. Don't count it
            // because we already did that when we had the sof && !eof
            // condition above.
            AIL_rx_buf_recycle(state);
            continue;
        }

        // It might be that the frame is larger than the users's maximum requested size,
        // because we have told H/W that the buffer length is the closest,
        // higher multiple of the platform's burst size. If so, count it and
        // discard it.
        if (dcb_status.fragment_size_bytes > state->rx_head_hw->buf_dscr.buf_size_bytes) {
            state->stati.rx_oversize_drops++;
            AIL_rx_buf_recycle(state);
            continue;
        }

        // Here, we have one frame in one Rx buffer.
        buf_dscr                  = state->rx_head_hw->buf_dscr;
        buf_dscr.frm_length_bytes = dcb_status.fragment_size_bytes;
        buf_dscr.result           = UFDMA_RC_OK;
        buf_dscr.timestamp        = TIMESTAMP();

        // Invalidate the data area that the FDMA received the frame data into
        DCACHE_INVALIDATE(buf_dscr.buf, buf_dscr.frm_length_bytes);

        // We also keep per-Rx queue statistics, which requires that we
        // know which Rx queue this frame was received on.
        UFDMA_CIL_FUNC_RC(state, rx_qu_mask_get, state->rx_head_hw, &rx_qu_mask);
        rx_qu = AIL_rx_qu_from_mask(rx_qu_mask);
        buf_dscr.rx_qu = rx_qu;

        if (rx_qu >= ARRSZ(state->stati.rx_frms)) {
            UFDMA_E("Invalid Rx queue (%u). Adjusting to %u", rx_qu, ARRSZ(state->stati.rx_frms));
            rx_qu = ARRSZ(state->stati.rx_frms) - 1;
        }

        UFDMA_RC(AIL_rx_throttle_suspend_check(state, rx_qu, buf_dscr.frm_length_bytes, &drop));

        if (drop) {
            // We have just entered the suspended state or are already in the suspended state
            // and are just reading out remaining frames from the H/W queue.
            state->stati.rx_suspended_drops++;
            AIL_rx_buf_recycle(state);
            continue;
        }

        // Now that we have invalidated the cache, we should ask the CIL
        // layer, whether this really is a frame we should forward to the
        // application. There may be many different chip-specific reasons
        // for not forwarding it.
        UFDMA_CIL_FUNC_RC(state, rx_frm_drop, state->rx_head_hw, &drop);
        if (drop) {
            // CIL wants us to drop it. Count it and do as it says.
            state->stati.rx_cil_drops++;
            AIL_rx_buf_recycle(state);
            continue;
        }

        // Some statistics
        state->stati.rx_callback_calls++;
        state->stati.rx_callback_bytes += buf_dscr.frm_length_bytes;
        state->stati.rx_frms[rx_qu]++;
        state->stati.rx_bytes[rx_qu] += buf_dscr.frm_length_bytes;

        // Print some bytes from it
        UFDMA_DG(RX, "Rx %u byte frame (incl. %u byte IFH and 4 byte FCS) on Rx queue %u", buf_dscr.frm_length_bytes, state->self->props.rx_ifh_size_bytes, rx_qu);
        UFDMA_DG_HEX(RX, buf_dscr.buf, buf_dscr.frm_length_bytes < 96 ? buf_dscr.frm_length_bytes : 96);

        // Call the network driver with his own buffer descriptor.
        // Notice that we must release all our own references to this
        // buffer prior to the callback call, because we hand over ownership
        // of it with the call.
        new_head = state->rx_head_hw->next;

        state->callout.rx_callback(state->self, &buf_dscr);

        state->rx_head_hw = new_head;
    }

    // Check if we need to restart the channel.
    UFDMA_RC(AIL_rx_restart(state));

    return UFDMA_RC_OK;
}

/**
 * AIL_tx_done()
 */
static int AIL_tx_done(ufdma_state_t *state)
{
    vtss_ufdma_buf_dscr_t buf_dscr;
    ufdma_hw_dcb_status_t dcb_status;
    ufdma_dcb_t           *new_head;

    UFDMA_DG(TX, "Enter");

    // Iterate over all DCBs handed over to H/W and stop when we reach one that has not yet been injected.
    while (state->tx_head_hw) {
        // See if the FDMA indeed has injected the current frame.

        // Get this DCB's status
        UFDMA_CIL_FUNC_RC(state, dcb_status_decode, state->tx_head_hw, &dcb_status, FALSE /* is_rx = FALSE */);
        UFDMA_DG(TX, "sof = %u, eof = %u, tx_done = %u",
                 dcb_status.sof,
                 dcb_status.eof,
                 dcb_status.tx_done);

        if (!dcb_status.tx_done) {
            break;
        }

        // We only support single-DCB frames
        if (!dcb_status.sof) {
            UFDMA_EG(TX, "SOF not set");
        }

        if (!dcb_status.eof) {
            UFDMA_EG(TX, "EOF not set");
        }

        buf_dscr = state->tx_head_hw->buf_dscr;
        buf_dscr.result = UFDMA_RC_OK;
        buf_dscr.timestamp = TIMESTAMP();

        // Some statistics
        state->stati.tx_done_callback_calls++;

        // Call the network driver with his own buffer descriptor.
        // Notice that we must release all our own references to this
        // buffer prio to the callback call, because we hand over ownership
        // of it with the call.
        new_head = state->tx_head_hw->next;

        state->callout.tx_callback(state->self, &buf_dscr);

        state->tx_head_hw = new_head;
    }

    UFDMA_DG(TX, "Exit");

    return UFDMA_RC_OK;
}

/**
 * AIL_callback_all()
 */
static void AIL_callback_all(ufdma_state_t *state, ufdma_dcb_t *head, void (*callback)(vtss_ufdma_platform_driver_t *self, vtss_ufdma_buf_dscr_t *buf_dscr), char *who)
{
    // Because the DCB state is located in memory that probably will get
    // freed during invokation by the rx_callback() function, we need the
    // next pointer before calling rx_callback().

    UFDMA_I("Freeing %s", who);
    while (head) {
        ufdma_dcb_t           *next    = head->next;
        vtss_ufdma_buf_dscr_t buf_dscr = head->buf_dscr;

        UFDMA_I("%s: Freeing %p", who, head);

        buf_dscr.result = UFDMA_RC_UNINIT;
        state->callout.rx_callback(state->self, &buf_dscr);

        head = next;
    }
}

/**
 * AIL_rx_buf_add()
 */
static int AIL_rx_buf_add(vtss_ufdma_platform_driver_t *self, vtss_ufdma_buf_dscr_t *rx_buf_dscr)
{
    ufdma_state_t *state;
    ufdma_dcb_t   *dcb;

    UFDMA_RC(AIL_self_chk(self, &state));

    UFDMA_RC(AIL_buf_dscr_chk(state, rx_buf_dscr, TRUE /* This is an Rx buffer */));

    dcb = UFDMA_dcb_prepare(self, rx_buf_dscr);

    // Link the DCB into the start of the current S/W list
    // and ask CIL to initialize the H/W area and link it to the current head.
    UFDMA_RC(AIL_rx_buf_add_do(state, dcb));

    // Start the FDMA if it currently doesn't have any buffers.
    // If it has, the poll() or rx_poll() function will start it.
    UFDMA_RC(AIL_rx_restart(state));

    state->stati.rx_buf_add_calls++;

    return UFDMA_RC_OK;
}

/**
 * AIL_rx_buf_alloc_size_get()
 */
static int AIL_rx_buf_alloc_size_get(vtss_ufdma_platform_driver_t *self, unsigned int wanted_size_bytes, unsigned int *needed_size_bytes)
{
    ufdma_state_t *state;
    unsigned int  extra, b, c;

    UFDMA_RC(AIL_self_chk(self, &state));

    if (needed_size_bytes == NULL) {
        return UFDMA_RC_ARG;
    }

    b = UFDMA_DCACHE_LINE_SIZE_BYTES - 1;
    c = state->cil.rx_burst_size_bytes - 1;

    if (b > c) {
        extra = b;
    } else {
        extra = c;
    }

    *needed_size_bytes = wanted_size_bytes + extra;

    UFDMA_DG(RX, "wanted = %u bytes, needed = %u bytes", wanted_size_bytes, *needed_size_bytes);

    return UFDMA_RC_OK;
}

/**
 * AIL_tx()
 */
static int AIL_tx(vtss_ufdma_platform_driver_t *self, vtss_ufdma_buf_dscr_t *tx_buf_dscr)
{
    ufdma_state_t *state;
    ufdma_dcb_t   *dcb;

    UFDMA_RC(AIL_self_chk(self, &state));
    UFDMA_RC(AIL_buf_dscr_chk(state, tx_buf_dscr, FALSE /* This is a Tx buffer */));

    dcb = UFDMA_dcb_prepare(self, tx_buf_dscr);

    // Ask CIL to initialize the H/W area and link it to the end of the current list.
    UFDMA_CIL_FUNC_RC(state, tx_dcb_init, dcb, state->tx_tail_sw);

    // Get the frame written to main memory
    DCACHE_FLUSH(dcb->buf_dscr.buf, dcb->buf_dscr.buf_size_bytes);

    // Link it in to the tail of currently pending frames.
    if (state->tx_tail_sw) {
        state->tx_tail_sw->next = dcb;
    } else {
        // When the tail is NULL, so is the head.
        state->tx_head_sw = dcb;
    }

    // The new DCB is the new tail.
    state->tx_tail_sw = dcb;

    UFDMA_DG(TX, "Tx %u byte frame (incl. %u byte IFH and 4 byte FCS)", tx_buf_dscr->buf_size_bytes, self->props.tx_ifh_size_bytes);
    UFDMA_DG_HEX(TX, tx_buf_dscr->buf, tx_buf_dscr->buf_size_bytes < 96 ? tx_buf_dscr->buf_size_bytes : 96);

    // Start the FDMA if it is not currently busy transmitting frames.
    // If it is, the poll() or tx_poll() function will re-start it.
    if (state->tx_head_hw == NULL) {
        UFDMA_IG(TX, "Starting Tx");
        UFDMA_CIL_FUNC_RC(state, tx_start, state->tx_head_sw);

        // Now that we know that the H/W could get started,
        // move the list from the S/W to the H/W list.
        state->tx_head_hw = state->tx_head_sw;
        state->tx_head_sw = NULL;
        state->tx_tail_sw = NULL;
    }

    state->stati.tx_calls++;
    state->stati.tx_bytes += dcb->buf_dscr.buf_size_bytes;

    return UFDMA_RC_OK;
}

/**
 * AIL_poll()
 */
static int AIL_poll(vtss_ufdma_platform_driver_t *self)
{
    ufdma_state_t *state;

    UFDMA_RC(AIL_self_chk(self, &state));

    UFDMA_CIL_FUNC_RC(state, poll);

    state->stati.poll_calls++;

    return UFDMA_RC_OK;
}

/**
 * AIL_rx_poll()
 */
static int AIL_rx_poll(vtss_ufdma_platform_driver_t *self)
{
    ufdma_state_t *state;

    UFDMA_RC(AIL_self_chk(self, &state));

    UFDMA_CIL_FUNC_RC(state, rx_poll);

    state->stati.rx_poll_calls++;

    return UFDMA_RC_OK;
}

/**
 * AIL_tx_poll()
 */
static int AIL_tx_poll(vtss_ufdma_platform_driver_t *self)
{
    ufdma_state_t *state;

    UFDMA_RC(AIL_self_chk(self, &state));

    UFDMA_CIL_FUNC_RC(state, tx_poll);

    state->stati.tx_poll_calls++;

    return UFDMA_RC_OK;
}

/**
 * AIL_uninit()
 */
static int AIL_uninit(vtss_ufdma_platform_driver_t *self)
{
    ufdma_state_t *state;

    UFDMA_RC(AIL_self_chk(self, &state));

    UFDMA_CIL_FUNC_RC(state, uninit);

    // Cycle through all Rx and Tx buffers and call back with an error code
    // indicating that they are returned because we are getting uninitialized.
    // We must run through both those already added to H/W and those not yet
    // added. It's safe to run through those added to H/W as well, because
    // the above CIL call has stopped and disabled the FDMA (gracefully).

    AIL_callback_all(state, state->rx_head_hw, state->callout.rx_callback, "rx_head_hw");
    AIL_callback_all(state, state->rx_head_sw, state->callout.rx_callback, "rx_head_sw");
    AIL_callback_all(state, state->tx_head_hw, state->callout.tx_callback, "tx_head_hw");
    AIL_callback_all(state, state->tx_head_sw, state->callout.tx_callback, "tx_head_sw");

    state->rx_head_hw = NULL;
    state->rx_head_sw = NULL;
    state->tx_head_hw = NULL;
    state->tx_head_sw = NULL;
    state->tx_tail_sw = NULL;

    UFDMA_RC(AIL_rx_qu_suspend_set_all(state, FALSE));

    state->initialized = FALSE;

    return UFDMA_RC_OK;
}

/**
 * AIL_debug_print()
 */
static int AIL_debug_print(vtss_ufdma_platform_driver_t *self, vtss_ufdma_debug_info_t *info, int (*pr)(const char *fmt, ...))
{
    ufdma_state_t *state;

    UFDMA_RC(AIL_self_chk(self, &state));

    if (info == NULL || pr == NULL) {
        return UFDMA_RC_ARG;
    }

    if (info->layer >= VTSS_UFDMA_DEBUG_LAYER_LAST || info->group >= VTSS_UFDMA_DEBUG_GROUP_LAST) {
        return UFDMA_RC_ARG;
    }

    if (info->layer == VTSS_UFDMA_DEBUG_LAYER_AIL || info->layer == VTSS_UFDMA_DEBUG_LAYER_ALL) {
        (void)pr("Application Interface Layer\n");
        (void)pr("---------------------------\n\n");
        UFDMA_RC(AIL_debug_print_do(state, info, pr));
        if (info->layer == VTSS_UFDMA_DEBUG_LAYER_ALL) {
            (void)pr("\n");
        }
    }

    if (info->layer == VTSS_UFDMA_DEBUG_LAYER_CIL || info->layer == VTSS_UFDMA_DEBUG_LAYER_ALL) {
        (void)pr("Chip Interface Layer\n");
        (void)pr("--------------------\n\n");
        UFDMA_CIL_FUNC_RC(state, debug_print, info, pr);
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_stati_clr()
 */
static int AIL_stati_clr(vtss_ufdma_platform_driver_t *self)
{
    ufdma_state_t *state;

    UFDMA_RC(AIL_self_chk(self, &state));

    memset(&state->stati, 0, sizeof(state->stati));

    return UFDMA_RC_OK;
}

/**
 * AIL_trace_level_set()
 */
static int AIL_trace_level_set(vtss_ufdma_platform_driver_t *self, vtss_ufdma_trace_layer_t layer, vtss_ufdma_trace_group_t group, vtss_ufdma_trace_level_t level)
{
    ufdma_state_t *state;
    u32           group_iter, group_min, group_max, layer_iter, layer_min, layer_max;

    UFDMA_RC(AIL_self_chk(self, &state));

    if (layer > VTSS_UFDMA_TRACE_LAYER_ALL ||
        group > VTSS_UFDMA_TRACE_GROUP_ALL ||
        level >= VTSS_UFDMA_TRACE_LEVEL_LAST) {
        return UFDMA_RC_ARG;
    }

    if (layer == VTSS_UFDMA_TRACE_LAYER_ALL) {
        layer_min = 0;
        layer_max = VTSS_UFDMA_TRACE_LAYER_ALL - 1;
    } else {
        layer_min = layer_max = layer;
    }

    if (group == VTSS_UFDMA_TRACE_GROUP_ALL) {
        group_min = 0;
        group_max = VTSS_UFDMA_TRACE_GROUP_ALL - 1;
    } else {
        group_min = group_max = group;
    }

    for (group_iter = group_min; group_iter <= group_max; group_iter++) {
        for (layer_iter = layer_min; layer_iter <= layer_max; layer_iter++) {
            state->trace_conf[group_iter].level[layer_iter] = level;
        }
    }

    return UFDMA_RC_OK;
}

/**
 * AIL_error_txt()
 */
static const char *AIL_error_txt(vtss_ufdma_platform_driver_t *self, int error_code)
{
    switch (error_code) {
    case UFDMA_RC_OK:
        return "No error";

    case UFDMA_RC_SELF:
        return "self is NULL";

    case UFDMA_RC_STATE:
        return "self->state is NULL";

    case UFDMA_RC_ARG:
        return "Invalid argument passed to function";

    case UFDMA_RC_INIT_CONF:
        return "Invalid value passed in vtss_fdma_init_conf_t structure";

    case UFDMA_RC_NO_SUCH_CIL_FUNC:
        return "Internal error: No such CIL function";

    case UFDMA_RC_NO_SUCH_AIL_FUNC:
        return "Internal error: No such AIL function";

    case UFDMA_RC_ALREADY_INITIALIZED:
        return "uFDMA is already initialized";

    case UFDMA_RC_NOT_INITIALIZED:
        return "uFDMA is not initialized";

    case UFDMA_RC_BUF_SIZE:
        return "Buffer size is invalid (must accommodate minimum Ethernet frame + IFH)";

    case UFDMA_RC_BUF_STATE:
        return "buf_dscr->buf_state is NULL";

    case UFDMA_RC_RX_BUF_ALIGNMENT:
        return "Rx buffers must be 32-bit aligned";

    case UFDMA_RC_UNINIT:
        return "Buffers are being returned because the uFDMA has been uninitialized";

    case UFDMA_RC_CIL:
        return "Unspecified CIL error";

    default:
        return "Unknown error code";
    }
}

/**
 * AIL_throttle_conf_get()
 */
static int AIL_throttle_conf_get(vtss_ufdma_platform_driver_t *self, vtss_ufdma_throttle_conf_t *throttle_conf)
{
    ufdma_state_t *state;

    UFDMA_RC(AIL_self_chk(self, &state));

    if (throttle_conf == NULL) {
        return UFDMA_RC_ARG;
    }

    *throttle_conf = state->throttle_conf;

    return UFDMA_RC_OK;
}

/**
 * AIL_throttle_conf_set()
 */
static int AIL_throttle_conf_set(vtss_ufdma_platform_driver_t *self, vtss_ufdma_throttle_conf_t *throttle_conf)
{
    ufdma_state_t          *state;
    ufdma_throttle_state_t *throttle_state;

    UFDMA_RC(AIL_self_chk(self, &state));

    if (throttle_conf == NULL) {
        return UFDMA_RC_ARG;
    }

    // Before changing the configuration, pull the Rx queues out of suspended state.
    UFDMA_RC(AIL_rx_qu_suspend_set_all(state, FALSE));

    throttle_state = &state->throttle_state;
    memset(throttle_state, 0, sizeof(*throttle_state));

    state->throttle_conf = *throttle_conf;

    return UFDMA_RC_OK;
}

/**
 * AIL_throttle_tick()
 */
static int AIL_throttle_tick(vtss_ufdma_platform_driver_t *self)
{
    ufdma_state_t          *state;
    ufdma_throttle_state_t *throttle_state;
    unsigned int           rx_qu;

    UFDMA_RC(AIL_self_chk(self, &state));

    throttle_state = &state->throttle_state;

    throttle_state->tick_cnt++;

    // Loop over all queues to see if any of them is currently suspended and needs unsuspension.
    for (rx_qu = 0; rx_qu < ARRSZ(throttle_state->ticks_left); rx_qu++) {
        if (throttle_state->ticks_left[rx_qu] > 0) {
            // The queue is suspended. Check to see if suspension time has elapsed.
            if (--throttle_state->ticks_left[rx_qu] == 0) {
                // The queue has been silent long enough. Re-enable it.
                UFDMA_CIL_FUNC_RC(state, rx_qu_suspend_set, rx_qu, FALSE);
            }
        }

        // Update max received and clear counter. We do it even if the queue is not enabled for throttling,
        // because it can give the user an idea of the maximum number of frames received between two ticks.
        if (throttle_state->tick_cnt > 2) {
            if (throttle_state->frm_cnt[rx_qu] > throttle_state->statistics_max_frames_per_tick[rx_qu]) {
                // Only update max when the application has started a steady throttle tick.
                throttle_state->statistics_max_frames_per_tick[rx_qu] = throttle_state->frm_cnt[rx_qu];
            }
            if (throttle_state->byte_cnt[rx_qu] > throttle_state->statistics_max_bytes_per_tick[rx_qu]) {
                // Only update max when the application has started a steady throttle tick.
                throttle_state->statistics_max_bytes_per_tick[rx_qu] = throttle_state->byte_cnt[rx_qu];
            }
        }

        throttle_state->frm_cnt[rx_qu] = 0;
        throttle_state->byte_cnt[rx_qu] = 0;
    }

    return UFDMA_RC_OK;
}

/******************************************************************************/
//
// Public functions
//
/******************************************************************************/

/**
 * vtss_ufdma_init()
 */
int vtss_ufdma_init(vtss_ufdma_platform_driver_t *self, vtss_ufdma_init_conf_t *init_conf, BOOL dual_chip, ufdma_state_t **pass_out_state)
{
    ufdma_state_t *state;
    int           grp, layer;

    UFDMA_RC(AIL_self_chk_no_init(self, &state));

    *pass_out_state = state;

    memset(state, 0, sizeof(*state));

    if (init_conf == NULL) {
        return UFDMA_RC_ARG;
    }

    // Check all function pointers in init_conf.
    if (init_conf->rx_callback      == NULL ||
        init_conf->tx_callback      == NULL ||
        init_conf->cache_flush      == NULL ||
        init_conf->cache_invalidate == NULL ||
        init_conf->virt_to_phys     == NULL ||
        init_conf->reg_read         == NULL ||
        init_conf->reg_write        == NULL) {
        return UFDMA_RC_INIT_CONF;
    }

    state->callout = *init_conf;

    // Register our own functions for use by the CIL layer - to avoid making them public (non-static)
    state->ail.rx_frm             = AIL_rx_frm;
    state->ail.rx_buf_recycle_all = AIL_rx_buf_recycle_all;
    state->ail.tx_done            = AIL_tx_done;
    state->ail.cpu_to_bus         = AIL_cpu_to_bus;
    state->ail.bus_to_cpu         = AIL_bus_to_cpu;
    state->ail.reorder_barrier    = AIL_reorder_barrier;
    state->ail.reg_rd             = AIL_reg_rd;
    state->ail.reg_wr             = AIL_reg_wr;

    // Default all API functions, but the init-function which is chip-specific.
    self->rx_buf_add              = AIL_rx_buf_add;
    self->rx_buf_alloc_size_get   = AIL_rx_buf_alloc_size_get;
    self->tx                      = AIL_tx;
    self->poll                    = AIL_poll;
    self->rx_poll                 = AIL_rx_poll;
    self->tx_poll                 = AIL_tx_poll;
    self->uninit                  = AIL_uninit;
    self->debug_print             = AIL_debug_print;
    self->stati_clr               = AIL_stati_clr;
    self->trace_level_set         = AIL_trace_level_set;
    self->error_txt               = AIL_error_txt;
    self->throttle_conf_get       = AIL_throttle_conf_get;
    self->throttle_conf_set       = AIL_throttle_conf_set;
    self->throttle_tick           = AIL_throttle_tick;

    // With the following, we can get from both state to self and self to state.
    state->self = self;

    // Initialize trace to all error
    for (grp = 0; grp < ARRSZ(state->trace_conf); grp++) {
        for (layer = 0; layer < ARRSZ(state->trace_conf[grp].level); layer++) {
            state->trace_conf[grp].level[layer] = VTSS_UFDMA_TRACE_LEVEL_ERROR;
        }
    }

    state->initialized = TRUE;

    return UFDMA_RC_OK;
}

