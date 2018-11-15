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

#ifndef _VTSS_LUTON26_REGS_UART_H_
#define _VTSS_LUTON26_REGS_UART_H_

#include "vtss_luton26_regs_common.h"

#define VTSS_UART_UART_RBR_THR               VTSS_IOREG(VTSS_TO_UART,0x0)
#define  VTSS_F_UART_UART_RBR_THR_RBR_THR(x)  VTSS_ENCODE_BITFIELD(x,0,8)
#define  VTSS_M_UART_UART_RBR_THR_RBR_THR     VTSS_ENCODE_BITMASK(0,8)
#define  VTSS_X_UART_UART_RBR_THR_RBR_THR(x)  VTSS_EXTRACT_BITFIELD(x,0,8)

#define VTSS_UART_UART_IER                   VTSS_IOREG(VTSS_TO_UART,0x1)
#define  VTSS_F_UART_UART_IER_PTIME           VTSS_BIT(7)
#define  VTSS_F_UART_UART_IER_EDSSI           VTSS_BIT(3)
#define  VTSS_F_UART_UART_IER_ELSI            VTSS_BIT(2)
#define  VTSS_F_UART_UART_IER_ETBEI           VTSS_BIT(1)
#define  VTSS_F_UART_UART_IER_ERBFI           VTSS_BIT(0)

#define VTSS_UART_UART_IIR_FCR               VTSS_IOREG(VTSS_TO_UART,0x2)
#define  VTSS_F_UART_UART_IIR_FCR_FIFOSE_RT(x)  VTSS_ENCODE_BITFIELD(x,6,2)
#define  VTSS_M_UART_UART_IIR_FCR_FIFOSE_RT     VTSS_ENCODE_BITMASK(6,2)
#define  VTSS_X_UART_UART_IIR_FCR_FIFOSE_RT(x)  VTSS_EXTRACT_BITFIELD(x,6,2)
#define  VTSS_F_UART_UART_IIR_FCR_TET(x)      VTSS_ENCODE_BITFIELD(x,4,2)
#define  VTSS_M_UART_UART_IIR_FCR_TET         VTSS_ENCODE_BITMASK(4,2)
#define  VTSS_X_UART_UART_IIR_FCR_TET(x)      VTSS_EXTRACT_BITFIELD(x,4,2)
#define  VTSS_F_UART_UART_IIR_FCR_XFIFOR      VTSS_BIT(2)
#define  VTSS_F_UART_UART_IIR_FCR_RFIFOR      VTSS_BIT(1)
#define  VTSS_F_UART_UART_IIR_FCR_FIFOE       VTSS_BIT(0)

#define VTSS_UART_UART_LCR                   VTSS_IOREG(VTSS_TO_UART,0x3)
#define  VTSS_F_UART_UART_LCR_DLAB            VTSS_BIT(7)
#define  VTSS_F_UART_UART_LCR_BC              VTSS_BIT(6)
#define  VTSS_F_UART_UART_LCR_EPS             VTSS_BIT(4)
#define  VTSS_F_UART_UART_LCR_PEN             VTSS_BIT(3)
#define  VTSS_F_UART_UART_LCR_STOP            VTSS_BIT(2)
#define  VTSS_F_UART_UART_LCR_DLS(x)          VTSS_ENCODE_BITFIELD(x,0,2)
#define  VTSS_M_UART_UART_LCR_DLS             VTSS_ENCODE_BITMASK(0,2)
#define  VTSS_X_UART_UART_LCR_DLS(x)          VTSS_EXTRACT_BITFIELD(x,0,2)

#define VTSS_UART_UART_MCR                   VTSS_IOREG(VTSS_TO_UART,0x4)
#define  VTSS_F_UART_UART_MCR_AFCE            VTSS_BIT(5)
#define  VTSS_F_UART_UART_MCR_LB              VTSS_BIT(4)
#define  VTSS_F_UART_UART_MCR_RTS             VTSS_BIT(1)

#define VTSS_UART_UART_LSR                   VTSS_IOREG(VTSS_TO_UART,0x5)
#define  VTSS_F_UART_UART_LSR_RFE             VTSS_BIT(7)
#define  VTSS_F_UART_UART_LSR_TEMT            VTSS_BIT(6)
#define  VTSS_F_UART_UART_LSR_THRE            VTSS_BIT(5)
#define  VTSS_F_UART_UART_LSR_BI              VTSS_BIT(4)
#define  VTSS_F_UART_UART_LSR_FE              VTSS_BIT(3)
#define  VTSS_F_UART_UART_LSR_PE              VTSS_BIT(2)
#define  VTSS_F_UART_UART_LSR_OE              VTSS_BIT(1)
#define  VTSS_F_UART_UART_LSR_DR              VTSS_BIT(0)

#define VTSS_UART_UART_MSR                   VTSS_IOREG(VTSS_TO_UART,0x6)
#define  VTSS_F_UART_UART_MSR_CTS             VTSS_BIT(4)
#define  VTSS_F_UART_UART_MSR_DCTS            VTSS_BIT(0)

#define VTSS_UART_UART_SCR                   VTSS_IOREG(VTSS_TO_UART,0x7)
#define  VTSS_F_UART_UART_SCR_SCR(x)          VTSS_ENCODE_BITFIELD(x,0,8)
#define  VTSS_M_UART_UART_SCR_SCR             VTSS_ENCODE_BITMASK(0,8)
#define  VTSS_X_UART_UART_SCR_SCR(x)          VTSS_EXTRACT_BITFIELD(x,0,8)

#define VTSS_UART_UART_USR                   VTSS_IOREG(VTSS_TO_UART,0x1f)
#define  VTSS_F_UART_UART_USR_BUSY            VTSS_BIT(0)


#endif /* _VTSS_LUTON26_REGS_UART_H_ */
