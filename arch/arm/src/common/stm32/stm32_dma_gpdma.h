/****************************************************************************
 * arch/arm/src/common/stm32/stm32_dma_gpdma.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_DMA_GPDMA_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_DMA_GPDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdint.h>

#include "hardware/stm32_gpdma.h"

#ifdef CONFIG_DEBUG_DMA_INFO
#  error "CONFIG_DEBUG_DMA_INFO not yet implemented."
#  undef CONFIG_DEBUG_DMA_INFO
#endif

#ifdef CONFIG_STM32_DMACAPABLE
#  error "CONFIG_STM32_DMACAPABLE not yet implemented."
#  undef CONFIG_STM32_DMACAPABLE
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These definitions provide the bit encoding of the 'status' parameter
 * passed to the DMA callback function (see dma_callback_t).
 */

#define DMA_STATUS_TCF        (1 << 0) /* Transfer Complete */
#define DMA_STATUS_HTF        (1 << 1) /* Half Transfer */
#define DMA_STATUS_DTEF       (1 << 2) /* Data transfer error */
#define DMA_STATUS_ULEF       (1 << 3) /* Update link transfer error */
#define DMA_STATUS_USEF       (1 << 4) /* User setting error */
#define DMA_STATUS_SUSPF      (1 << 5) /* Completed suspension flag */
#define DMA_STATUS_TOF        (1 << 6) /* Trigger overrun flag */

#define DMA_STATUS_FATAL      (DMA_STATUS_DTEF | DMA_STATUS_ULEF | DMA_STATUS_USEF)
#define DMA_STATUS_SUCCESS    (DMA_STATUS_TCF | DMA_STATUS_HTF)

/* GPDMA Mode Flags: WARNING!! NOT YET IMPLEMENTED! */

#define GPDMACFG_MODE_CIRC    (1 << 0)  /* Enable Circular mode */
#define GPDMACFG_MODE_PFC     (1 << 1)  /* Enable Peripheral flow control */
#define GPDMACFG_MODE_DB      (1 << 2)  /* Enable Double buffer mode */
#define GPDMACFG_MODE_TRIG    (1 << 3)  /* Use trigsel, trigpol and trigm */
#define GPDMACFG_MODE_LAP1    (1 << 6)  /* Fetch LLIs over port 1 (LAP) */

/* Transfer complete event mode (TCEM): when the TC and HT events fire.
 * For a linked list, use GPDMACFG_MODE_TCEM_LLI for one event per LLI,
 * or GPDMACFG_MODE_TCEM_CHAN for one event at the end of the chain.
 */

#define GPDMACFG_MODE_TCEM_SHIFT    (4)
#define GPDMACFG_MODE_TCEM_MASK     (3 << GPDMACFG_MODE_TCEM_SHIFT)
#  define GPDMACFG_MODE_TCEM_BLOCK  (0 << GPDMACFG_MODE_TCEM_SHIFT)
#  define GPDMACFG_MODE_TCEM_2DBLOCK (1 << GPDMACFG_MODE_TCEM_SHIFT)
#  define GPDMACFG_MODE_TCEM_LLI    (2 << GPDMACFG_MODE_TCEM_SHIFT)
#  define GPDMACFG_MODE_TCEM_CHAN   (3 << GPDMACFG_MODE_TCEM_SHIFT)

/* Trigger polarity, for the trigpol field */

#define GPDMACFG_TRIGPOL_RISING  (1)   /* Trigger on rising edge */
#define GPDMACFG_TRIGPOL_FALLING (2)   /* Trigger on falling edge */

/* Trigger mode, for the trigm field: what one trigger hit allows */

#define GPDMACFG_TRIGM_BLOCK     (0)   /* First burst of each block */
#define GPDMACFG_TRIGM_2DBLOCK   (1)   /* First burst of each 2D/repeated block */
#define GPDMACFG_TRIGM_LLI       (2)   /* Each LLI link transfer */
#define GPDMACFG_TRIGM_BURST     (3)   /* Each programmed burst */

/* Channel priority level
 * Refer to PRIO field in GPDMA_CxCR register description (RM0481)
 */

#define GPDMACFG_PRIO_LL      (0)   /* Low priority, low weight */
#define GPDMACFG_PRIO_LM      (1)   /* Low priority, mid weight */
#define GPMDACFG_PRIO_LH      (2)   /* Low priority, high weight */
#define GPDMACFG_PRIO_H       (3)   /* High priority */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPDMA transfer type enumeration */

enum gpdma_ttype_e
{
  /* Peripheral-to-memory transfer */

  GPDMA_TTYPE_P2M = 0,

  /* Memory-to-peripheral transfer */

  GPDMA_TTYPE_M2P,

  /* Memory-to-memory transfer, linear addressing */

  GPDMA_TTYPE_M2M_LINEAR,

  /* 2D Addressing needed (NOT IMPLEMENTED YET) */

  GPDMA_TTYPE_2D
};

#ifdef CONFIG_DEBUG_DMA_INFO
struct stm32_gpdma_reg_s
{
  uint32_t cxlbar;
  uint32_t cxfcr;
  uint32_t cxsr;
  uint32_t cxcr;
  uint32_t cxtr1;
  uint32_t cxtr2;
  uint32_t cxbr1;
  uint32_t cxsar;
  uint32_t cxdar;
  uint32_t cxtr3;
  uint32_t cxbr2;
  uint32_t cxllr;
};
#endif

struct stm32_gpdma_cfg_s
{
  uint32_t src_addr;
  uint32_t dest_addr;

  /* CxTR1 register for specified channel. */

  uint32_t tr1;

  /* request: Accepts GPDMA_CXTR2_SWREQ, GPDMA_CXTR2_DREQ, and
   * GPDMA_CXTR2_REQSEL(r) for r given by GPDMA_REQ_x
   * macros defined in the family hardware/stm32_dmasigmap.h
   */

  uint16_t request;

  /* Number of transfers, in units of the data width
   * specified in tr1.
   */

  uint16_t ntransfers;

  /* Priority level: refer to GPDMACFG_PRIO defines above */

  uint8_t  priority;

  /* mode flags, refer to GPDMACFG_MODE_X defines above. */

  uint8_t  mode;

  /* Hardware trigger, used only if mode has GPDMACFG_MODE_TRIG.
   * trigsel is a GPDMA_TRIG_x value from hardware/stm32_dmasigmap.h,
   * trigpol a GPDMACFG_TRIGPOL_x value and trigm a GPDMACFG_TRIGM_x value.
   */

  uint8_t  trigsel;
  uint8_t  trigpol;
  uint8_t  trigm;
};

/* One linked-list item (LLI), in the layout the GPDMA loads it from
 * memory when all of UT1, UT2, UB1, USA, UDA and ULL are set.  Fill it
 * with stm32_dmalli_init().  Each LLI must be word aligned and must not
 * cross a 64 KiB boundary, and all LLIs of a chain must be in the same
 * 64 KiB segment.
 */

struct stm32_gpdma_lli_s
{
  uint32_t tr1;   /* GPDMA_CxTR1 */
  uint32_t tr2;   /* GPDMA_CxTR2 */
  uint32_t br1;   /* GPDMA_CxBR1 (BNDT, bytes of source data) */
  uint32_t sar;   /* GPDMA_CxSAR */
  uint32_t dar;   /* GPDMA_CxDAR */
  uint32_t llr;   /* GPDMA_CxLLR (update bits and next LLI offset) */
};

/* DMA_HANDLE Provides an opaque reference that can be used to represent a
 * DMA stream.
 */

typedef void *DMA_HANDLE;

/* Description:
 *   This is the type of the callback that is used to inform the user of the
 *   completion of the DMA.  NOTE:  The DMA module does *NOT* perform any
 *   cache operations.  It is the responsibility of the DMA client to
 *   invalidate DMA buffers after completion of the DMA RX operations.
 *
 * Input Parameters:
 *   handle - Refers to the DMA channel or stream
 *   status - A bit encoded value that provides the completion status.  See
 *            the DMA_STATUS_* definitions above.
 *   arg    - A user-provided value that was provided when stm32_dmastart()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel based on provided transfer type. The driver will
 *   return the first available DMA channel compatible with the transfer type
 *
 * Input Parameters:
 *   type - Type of DMA transfer required
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(enum gpdma_ttype_e type);

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until stm32_dmachannel() is called again to re-gain access to the
 *   channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void stm32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *   cfg    - Transfer configuration
 *
 * Returned Value:
 *   OK on success, -EINVAL if the trigger configuration is not valid.
 *
 ****************************************************************************/

int stm32_dmasetup(DMA_HANDLE handle, struct stm32_gpdma_cfg_s *cfg);

/****************************************************************************
 * Name: stm32_dmalli_init
 *
 * Description:
 *   Fill one LLI from a transfer configuration.  src_addr, dest_addr, tr1,
 *   request, ntransfers, the trigger fields and the TCEM bits of mode are
 *   used; priority and the other mode bits are not.
 *
 * Input Parameters:
 *   lli  - The LLI to fill
 *   cfg  - Transfer configuration of this LLI
 *   next - The LLI to load when this one is done, or NULL if this is the
 *          last one.  It may be lli itself, or any earlier LLI, to loop.
 *
 * Returned Value:
 *   OK on success.  -EINVAL if an address is not aligned to its data width,
 *   ntransfers is zero or not a multiple of the source data width (and, if
 *   PAM packs, of the destination data width), the trigger is not valid,
 *   lli or next is not word aligned, or lli and next are not in the same
 *   64 KiB segment.
 *
 ****************************************************************************/

int stm32_dmalli_init(struct stm32_gpdma_lli_s *lli,
                      const struct stm32_gpdma_cfg_s *cfg,
                      const struct stm32_gpdma_lli_s *next);

/****************************************************************************
 * Name: stm32_dmasetup_lli
 *
 * Description:
 *   Configure the channel to run a chain of LLIs.  head is loaded into the
 *   channel registers and the rest is fetched by the GPDMA as it goes.
 *   The LLIs are not copied: they must not change until the channel is
 *   stopped or has completed.  The chain is checked before the channel is
 *   programmed.
 *
 * Input Parameters:
 *   handle   - DMA channel handle
 *   head     - First LLI of the chain
 *   priority - GPDMACFG_PRIO_x
 *   mode     - 0 or GPDMACFG_MODE_LAP1.  The other bits are ignored.
 *
 * Returned Value:
 *   OK on success, -EINVAL if the chain is not valid.  The channel is not
 *   changed on error.
 *
 ****************************************************************************/

int stm32_dmasetup_lli(DMA_HANDLE handle,
                       const struct stm32_gpdma_lli_s *head,
                       uint8_t priority, uint8_t mode);

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer.  NOTE:  The DMA module does *NOT* perform any
 *   cache operations.  It is the responsibility of the DMA client to clean
 *   DMA buffers after starting of the DMA TX operations.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half);

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address with the given configuration. This depends on the internal
 *   connections in the ARM bus matrix of the processor. Note that this only
 *   applies to memory addresses, it will return false for any peripheral
 *   address.
 *
 * Input Parameters:
 *
 *   maddr - starting memory address
 *   count - number of unit8 or uint16 or uint32 items as defined by MSIZE
 *           of ccr.
 *   ccr   - DMA stream configuration register
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_DMACAPABLE
bool stm32_dmacapable(DMA_HANDLE handle, struct stm32_gpdma_cfg_s *cfg);
#else
#  define stm32_dmacapable(handle, cfg) (true)
#endif

/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Input Parameters:
 *   TODO: Figure these out!! Not sure if I need the struct like F7 or not?
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs);
#else
#  define stm32_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmadump(DMA_HANDLE handle, const char *msg);
#else
#  define stm32_dmadump(handle,msg)
#endif

#if defined(__cplusplus)
}
#endif

#endif /* !__ASSEMBLY__*/
#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_DMA_GPDMA_H*/
