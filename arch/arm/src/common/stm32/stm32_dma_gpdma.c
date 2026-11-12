/****************************************************************************
 * arch/arm/src/common/stm32/stm32_dma_gpdma.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include "arm_internal.h"
#include "sched/sched.h"
#include "stm32_dma.h"
#include "hardware/stm32_gpdma.h"
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For GPDMA peripheral, each channel has channel specific addresses that are
 * at a base offset based on channel. Channel reg references will be based
 * off this in this file.
 */

#define CH_BASE_OFFSET(ch)  (0x80*(ch))
#define CH_CXLBAR_OFFSET     0x50
#define CH_CXFCR_OFFSET      0x5C
#define CH_CXSR_OFFSET       0x60
#define CH_CXCR_OFFSET       0x64
#define CH_CXTR1_OFFSET      0x90
#define CH_CXTR2_OFFSET      0x94
#define CH_CXBR1_OFFSET      0x98
#define CH_CXSAR_OFFSET      0x9C
#define CH_CXDAR_OFFSET      0xA0
#define CH_CXTR3_OFFSET      0xA4
#define CH_CXBR2_OFFSET      0xA8
#define CH_CXLLR_OFFSET      0xCC

/* Security attributes of the source and destination transfers */

#ifdef CONFIG_STM32_HAVE_GPDMA_SECURE
#  define GPDMA_CXTR1_SECATTR  (GPDMA_CXTR1_SSEC | GPDMA_CXTR1_DSEC)
#else
#  define GPDMA_CXTR1_SECATTR  0
#endif

/* LLR update bits of a full linear-addressing LLI */

#define GPDMA_LLR_UPDATE_ALL (GPDMA_CXLLR_UT1 | GPDMA_CXLLR_UT2 | \
                              GPDMA_CXLLR_UB1 | GPDMA_CXLLR_USA | \
                              GPDMA_CXLLR_UDA | GPDMA_CXLLR_ULL)

/* All LLIs of a channel are in the 64 KiB segment given by LBAR */

#define GPDMA_SEGMENT_MASK   GPDMA_CXLBAR_LBA_MASK
#define GPDMA_SAME_SEGMENT(a, b) \
  ((((uintptr_t)(a) ^ (uintptr_t)(b)) & GPDMA_SEGMENT_MASK) == 0)

/* Time allowed for the channel to suspend, and then to reset */

#define GPDMA_ABORT_TIMEOUT_US 1000

/* Most LLIs a 64 KiB segment can hold */

#define GPDMA_LLI_MAX        (65536 / sizeof(struct stm32_gpdma_lli_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpdma_ch_s
{
  uint8_t            dma_instance; /* GPDMA1 or GPDMA2 */
  uint8_t            channel;
  uint8_t            irq;
  enum gpdma_ttype_e type;
  bool               free;         /* Is this channel free to use. */
  bool               chain;        /* Set up by stm32_dmasetup_lli() */
  uint32_t           base;         /* Channel base address */
  dma_callback_t     callback;
  void              *arg;
  struct stm32_gpdma_cfg_s cfg;   /* Configuration passed at channel setup */

  /* LLIs of circular mode.  The alignment keeps both in one 64 KiB
   * segment.
   */

  struct stm32_gpdma_lli_s lli[2] aligned_data(64);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t gpdmach_getreg(struct gpdma_ch_s *chan,
                                      uint32_t offset);
static inline void gpdmach_putreg(struct gpdma_ch_s *chan, uint32_t offset,
                                  uint32_t value);
static inline void gpdmach_modifyreg32(struct gpdma_ch_s *chan,
                                       uint32_t offset, uint32_t clrbits,
                                       uint32_t setbits);
static int gpdma_ch_wait(struct gpdma_ch_s *chan, uint32_t offset,
                         uint32_t mask, uint32_t value);
static void gpdma_ch_abort(struct gpdma_ch_s *chan);
static void gpdma_ch_disable(struct gpdma_ch_s *chan);

static int gpdma_tr2(const struct stm32_gpdma_cfg_s *cfg, uint32_t *tr2);
static int gpdma_check_block(uint32_t tr1, uint32_t br1, uint32_t sar,
                             uint32_t dar);
static int gpdma_check_lli_addr(const struct stm32_gpdma_lli_s *lli);
static void gpdma_set_cr(struct gpdma_ch_s *chan, uint8_t priority,
                         uint8_t mode);
static int gpdma_setup(struct gpdma_ch_s *chan,
                       struct stm32_gpdma_cfg_s *cfg);
static int gpdma_setup_circular(struct gpdma_ch_s *chan,
                                struct stm32_gpdma_cfg_s *cfg);
static int gpdma_dmainterrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* One entry per implemented GPDMA channel.  Each family provides
 * STM32_DMAx_NCHAN and STM32_DMAx_IRQ_FIRST for its instances.
 */

#define GPDMA_CHAN(inst, ch) \
  { \
    .dma_instance = (inst), \
    .channel      = (ch), \
    .irq          = STM32_DMA##inst##_IRQ_FIRST + (ch), \
    .free         = true, \
    .base         = STM32_DMA##inst##_BASE + CH_BASE_OFFSET(ch) \
  }

static struct gpdma_ch_s g_chan[] =
{
#ifdef CONFIG_STM32_DMA1
#  if STM32_DMA1_NCHAN > 0
  GPDMA_CHAN(1, 0),
#  endif
#  if STM32_DMA1_NCHAN > 1
  GPDMA_CHAN(1, 1),
#  endif
#  if STM32_DMA1_NCHAN > 2
  GPDMA_CHAN(1, 2),
#  endif
#  if STM32_DMA1_NCHAN > 3
  GPDMA_CHAN(1, 3),
#  endif
#  if STM32_DMA1_NCHAN > 4
  GPDMA_CHAN(1, 4),
#  endif
#  if STM32_DMA1_NCHAN > 5
  GPDMA_CHAN(1, 5),
#  endif
#  if STM32_DMA1_NCHAN > 6
  GPDMA_CHAN(1, 6),
#  endif
#  if STM32_DMA1_NCHAN > 7
  GPDMA_CHAN(1, 7),
#  endif
#  if STM32_DMA1_NCHAN > 8
  GPDMA_CHAN(1, 8),
#  endif
#  if STM32_DMA1_NCHAN > 9
  GPDMA_CHAN(1, 9),
#  endif
#  if STM32_DMA1_NCHAN > 10
  GPDMA_CHAN(1, 10),
#  endif
#  if STM32_DMA1_NCHAN > 11
  GPDMA_CHAN(1, 11),
#  endif
#  if STM32_DMA1_NCHAN > 12
  GPDMA_CHAN(1, 12),
#  endif
#  if STM32_DMA1_NCHAN > 13
  GPDMA_CHAN(1, 13),
#  endif
#  if STM32_DMA1_NCHAN > 14
  GPDMA_CHAN(1, 14),
#  endif
#  if STM32_DMA1_NCHAN > 15
  GPDMA_CHAN(1, 15),
#  endif
#endif
#ifdef CONFIG_STM32_DMA2
#  if STM32_DMA2_NCHAN > 0
  GPDMA_CHAN(2, 0),
#  endif
#  if STM32_DMA2_NCHAN > 1
  GPDMA_CHAN(2, 1),
#  endif
#  if STM32_DMA2_NCHAN > 2
  GPDMA_CHAN(2, 2),
#  endif
#  if STM32_DMA2_NCHAN > 3
  GPDMA_CHAN(2, 3),
#  endif
#  if STM32_DMA2_NCHAN > 4
  GPDMA_CHAN(2, 4),
#  endif
#  if STM32_DMA2_NCHAN > 5
  GPDMA_CHAN(2, 5),
#  endif
#  if STM32_DMA2_NCHAN > 6
  GPDMA_CHAN(2, 6),
#  endif
#  if STM32_DMA2_NCHAN > 7
  GPDMA_CHAN(2, 7),
#  endif
#  if STM32_DMA2_NCHAN > 8
  GPDMA_CHAN(2, 8),
#  endif
#  if STM32_DMA2_NCHAN > 9
  GPDMA_CHAN(2, 9),
#  endif
#  if STM32_DMA2_NCHAN > 10
  GPDMA_CHAN(2, 10),
#  endif
#  if STM32_DMA2_NCHAN > 11
  GPDMA_CHAN(2, 11),
#  endif
#  if STM32_DMA2_NCHAN > 12
  GPDMA_CHAN(2, 12),
#  endif
#  if STM32_DMA2_NCHAN > 13
  GPDMA_CHAN(2, 13),
#  endif
#  if STM32_DMA2_NCHAN > 14
  GPDMA_CHAN(2, 14),
#  endif
#  if STM32_DMA2_NCHAN > 15
  GPDMA_CHAN(2, 15),
#  endif
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t gpdmach_getreg(struct gpdma_ch_s *chan,
                                      uint32_t offset)
{
  return getreg32(chan->base + offset);
}

static inline void gpdmach_putreg(struct gpdma_ch_s *chan, uint32_t offset,
                                  uint32_t value)
{
  putreg32(value, chan->base + offset);
}

static inline void gpdmach_modifyreg32(struct gpdma_ch_s *chan,
                                       uint32_t offset, uint32_t clrbits,
                                       uint32_t setbits)
{
  modifyreg32(chan->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: gpdma_dmainterrupt
 *
 * Description:
 *   DMA interrupt handler.
 *
 ****************************************************************************/

static int gpdma_dmainterrupt(int irq, void *context, void *arg)
{
  struct gpdma_ch_s *chan = NULL;
  uint32_t status;
  int i;

  /* Get the channel that generated the interrupt */

  for (i = 0; i < sizeof(g_chan) / sizeof(struct gpdma_ch_s); i++)
    {
      if (g_chan[i].irq == irq)
        {
          chan = &g_chan[i];
          break;
        }
    }

  if (chan == NULL)
    {
      DEBUGPANIC();
      return -EINVAL;
    }

  /* Get the interrupt status for this channel */

  status = (gpdmach_getreg(chan, CH_CXSR_OFFSET) >> 8) & 0x7f;

  /* Clear the fetched channel interrupts by setting bits in the flag
   * clear register
   */

  gpdmach_putreg(chan, CH_CXFCR_OFFSET, ~0);

  /* Invoke the callback */

  if (chan->callback)
    {
      chan->callback(chan, (uint8_t)status, chan->arg);
    }

  return 0;
}

/****************************************************************************
 * Name: gpdma_ch_wait
 *
 * Description:
 *   Wait until (register & mask) == value, or GPDMA_ABORT_TIMEOUT_US.
 *
 ****************************************************************************/

static int gpdma_ch_wait(struct gpdma_ch_s *chan, uint32_t offset,
                         uint32_t mask, uint32_t value)
{
  int i;

  for (i = 0; i < GPDMA_ABORT_TIMEOUT_US; i++)
    {
      if ((gpdmach_getreg(chan, offset) & mask) == value)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: gpdma_ch_abort
 *
 * Description:
 *   For the given channel, suspend and abort any ongoing channel transfers,
 *   as in the "channel abort" sequence of RM0481.  Returns after the abort
 *   has taken effect, or after a timeout.
 *
 ****************************************************************************/

static void gpdma_ch_abort(struct gpdma_ch_s *chan)
{
  if ((gpdmach_getreg(chan, CH_CXCR_OFFSET) & GPDMA_CXCR_EN) == 0)
    {
      return;
    }

  /* 1. Suspend.  This also clears the interrupt enables. */

  gpdmach_putreg(chan, CH_CXCR_OFFSET, GPDMA_CXCR_SUSP);

  /* 2. Wait for the channel to be idle: suspended once the ongoing burst
   *    or link transfer is done, or disabled if it completed meanwhile, in
   *    which case SUSPF is never set.
   */

  if (gpdma_ch_wait(chan, CH_CXSR_OFFSET, GPDMA_CXSR_IDLEF,
                    GPDMA_CXSR_IDLEF) < 0)
    {
      dmaerr("GPDMA%d CH%d: suspend timeout\n",
             chan->dma_instance, chan->channel);
    }

  /* 3. Reset the channel: this clears EN and SUSP */

  gpdmach_putreg(chan, CH_CXCR_OFFSET, GPDMA_CXCR_RESET);

  if (gpdma_ch_wait(chan, CH_CXCR_OFFSET,
                    GPDMA_CXCR_EN | GPDMA_CXCR_SUSP, 0) < 0)
    {
      dmaerr("GPDMA%d CH%d: reset timeout\n",
             chan->dma_instance, chan->channel);
    }
}

/****************************************************************************
 * Name: gpdma_ch_disable
 *
 * Description:
 *   Disable the DMA channel.
 *
 ****************************************************************************/

static void gpdma_ch_disable(struct gpdma_ch_s *chan)
{
  DEBUGASSERT(chan != NULL);

  gpdma_ch_abort(chan);

  /* Disable and clear all interrupts. */

  gpdmach_modifyreg32(chan, CH_CXCR_OFFSET, GPDMA_CXCR_ALLINTS, 0);
  gpdmach_modifyreg32(chan, CH_CXFCR_OFFSET, 0, ~0);
}

/****************************************************************************
 * Name: gpdma_tr2
 *
 * Description:
 *   Build the request and trigger fields of CxTR2 from a configuration.
 *
 ****************************************************************************/

static int gpdma_tr2(const struct stm32_gpdma_cfg_s *cfg, uint32_t *tr2)
{
  uint32_t reg;

  reg = (uint32_t)cfg->request &
        (GPDMA_CXTR2_REQSEL_MASK | GPDMA_CXTR2_DREQ | GPDMA_CXTR2_SWREQ);

  if ((cfg->mode & GPDMACFG_MODE_TRIG) != 0)
    {
      if ((cfg->trigpol != GPDMACFG_TRIGPOL_RISING &&
           cfg->trigpol != GPDMACFG_TRIGPOL_FALLING) ||
          cfg->trigsel > (GPDMA_CXTR2_TRIGSEL_MASK >>
                          GPDMA_CXTR2_TRIGSEL_SHIFT) ||
          cfg->trigm > GPDMACFG_TRIGM_BURST)
        {
          return -EINVAL;
        }

      reg |= GPDMA_CXTR2_TRIGSEL(cfg->trigsel) |
             ((uint32_t)cfg->trigpol << GPDMA_CXTR2_TRIGPOL_SHIFT) |
             ((uint32_t)cfg->trigm << GPDMA_CXTR2_TRIGM_SHIFT);
    }

  reg |= ((uint32_t)(cfg->mode & GPDMACFG_MODE_TCEM_MASK) >>
          GPDMACFG_MODE_TCEM_SHIFT) << GPDMA_CXTR2_TCEM_SHIFT;

  *tr2 = reg;
  return OK;
}

/****************************************************************************
 * Name: gpdma_check_block
 *
 * Description:
 *   Check the data widths, the addresses and BNDT of one block against
 *   the rules of the CxTR1 and CxBR1 descriptions in RM0481, so that the
 *   channel does not stop with a user setting error.
 *
 ****************************************************************************/

static int gpdma_check_block(uint32_t tr1, uint32_t br1, uint32_t sar,
                             uint32_t dar)
{
  uint32_t sdw = (tr1 & GPDMA_CXTR1_SDW_LOG2_MASK) >>
                 GPDMA_CXTR1_SDW_LOG2_SHIFT;
  uint32_t ddw = (tr1 & GPDMA_CXTR1_DDW_LOG2_MASK) >>
                 GPDMA_CXTR1_DDW_LOG2_SHIFT;
  uint32_t bndt = br1 & GPDMA_CXBR1_BNDT_MASK;

  /* 8-byte widths are not supported */

  if (sdw > 2 || ddw > 2)
    {
      return -EINVAL;
    }

  /* BNDT counts bytes of source data: a multiple of the source width */

  if (bndt == 0 || (bndt & ((1 << sdw) - 1)) != 0)
    {
      return -EINVAL;
    }

  /* When packing, BNDT must also be a multiple of the destination width */

  if (sdw != ddw && (tr1 & (2 << GPDMA_CXTR1_PAM_SHIFT)) != 0 &&
      (bndt & ((1 << ddw) - 1)) != 0)
    {
      return -EINVAL;
    }

  if ((sar & ((1 << sdw) - 1)) != 0 || (dar & ((1 << ddw) - 1)) != 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: gpdma_check_lli_addr
 *
 * Description:
 *   An LLI must be word aligned and must not cross a 64 KiB boundary.
 *
 ****************************************************************************/

static int gpdma_check_lli_addr(const struct stm32_gpdma_lli_s *lli)
{
  if (lli == NULL || ((uintptr_t)lli & 3) != 0 ||
      !GPDMA_SAME_SEGMENT(lli, (uintptr_t)lli + sizeof(*lli) - 1))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: gpdma_set_cr
 *
 * Description:
 *   Set the priority and the linked-list port.  Link step mode is not
 *   used.  The channel must be disabled.
 *
 ****************************************************************************/

static void gpdma_set_cr(struct gpdma_ch_s *chan, uint8_t priority,
                         uint8_t mode)
{
  uint32_t setbits = ((uint32_t)priority << GPDMA_CXCR_PRIO_SHIFT) &
                     GPDMA_CXCR_PRIO_MASK;

  if ((mode & GPDMACFG_MODE_LAP1) != 0)
    {
      setbits |= GPDMA_CXCR_LAP;
    }

  gpdmach_modifyreg32(chan, CH_CXCR_OFFSET,
                      GPDMA_CXCR_PRIO_MASK | GPDMA_CXCR_LAP | GPDMA_CXCR_LSM,
                      setbits);
}

/****************************************************************************
 * Name: gpdma_setup
 *
 * Assumptions:
 *   - EN bit not set. Channel must have been aborted before this is called.
 *
 ****************************************************************************/

static int gpdma_setup(struct gpdma_ch_s *chan,
                       struct stm32_gpdma_cfg_s *cfg)
{
  uint32_t reg;
  int ret;

  ret = gpdma_tr2(cfg, &reg);
  if (ret < 0)
    {
      return ret;
    }

  /* Make sure not to use linked list mode. */

  gpdmach_modifyreg32(chan, CH_CXLLR_OFFSET, ~0, 0);
  gpdmach_putreg(chan, CH_CXLBAR_OFFSET, 0);

  /* Set source and destination addresses. */

  gpdmach_putreg(chan, CH_CXSAR_OFFSET, cfg->src_addr);
  gpdmach_putreg(chan, CH_CXDAR_OFFSET, cfg->dest_addr);

  /* Set the channel priority according to configuration. */

  gpdma_set_cr(chan, cfg->priority, cfg->mode);

  /* Set channels TR1 register based on configuration provided. */

  gpdmach_putreg(chan, CH_CXTR1_OFFSET, cfg->tr1 | GPDMA_CXTR1_SECATTR);

  /* Request and trigger, assembled above */

  gpdmach_putreg(chan, CH_CXTR2_OFFSET, reg);

  /* Calculate block number of data bytes to transfer, update BR1 */

  reg = cfg->ntransfers;

  gpdmach_putreg(chan, CH_CXBR1_OFFSET, reg);

  return 0;
}

/****************************************************************************
 * Name: gpdma_setup_circular
 *
 * Description:
 *   Circular DMA requires linked list items (LLI) to setup properly. This
 *   function handles LLI allocation and handling necessary to implement
 *   circular DMA.
 *
 ****************************************************************************/

static int gpdma_setup_circular(struct gpdma_ch_s *chan,
                                struct stm32_gpdma_cfg_s *cfg)
{
  struct stm32_gpdma_lli_s *lli = chan->lli;
  uint32_t tr2;
  int ret;

  ret = gpdma_tr2(cfg, &tr2);
  if (ret < 0)
    {
      return ret;
    }

  lli[0].tr1 = cfg->tr1 | GPDMA_CXTR1_SECATTR;
  lli[0].tr2 = (2U << GPDMA_CXTR2_TCEM_SHIFT)
             | (tr2 & ~(GPDMA_CXTR2_DREQ | GPDMA_CXTR2_SWREQ |
                        GPDMA_CXTR2_TCEM_MASK));
  lli[0].br1 = cfg->ntransfers;
  lli[0].sar = cfg->src_addr;
  lli[0].dar = cfg->dest_addr;
  lli[0].llr = (GPDMA_CXLLR_UT1  /* reload TR1 */
             | GPDMA_CXLLR_UT2   /* reload TR2 */
             | GPDMA_CXLLR_UB1   /* reload BR1 */
             | GPDMA_CXLLR_USA   /* reload SAR */
             | GPDMA_CXLLR_UDA   /* reload DAR */
             | GPDMA_CXLLR_ULL)  /* reload LLR */
             | (((uint32_t)&lli[1]) & GPDMA_CXLLR_LA_MASK);

  lli[1].tr1 = lli[0].tr1;
  lli[1].tr2 = lli[0].tr2;
  lli[1].br1 = lli[0].br1;
  lli[1].sar = lli[0].sar;
  lli[1].dar = lli[0].dar;
  lli[1].llr = (lli[0].llr & ~GPDMA_CXLLR_LA_MASK)
             | (((uint32_t)&lli[0]) & GPDMA_CXLLR_LA_MASK);

  gpdmach_putreg(chan, CH_CXSAR_OFFSET,   lli[0].sar);
  gpdmach_putreg(chan, CH_CXDAR_OFFSET,   lli[0].dar);
  gpdmach_putreg(chan, CH_CXTR1_OFFSET,   lli[0].tr1);
  gpdmach_putreg(chan, CH_CXTR2_OFFSET,   lli[0].tr2);
  gpdmach_putreg(chan, CH_CXBR1_OFFSET,   lli[0].br1);
  gpdmach_putreg(chan, CH_CXLBAR_OFFSET,  (uint32_t)&lli[0]);
  gpdmach_putreg(chan, CH_CXLLR_OFFSET,   lli[0].llr);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  struct gpdma_ch_s *chan;
  int i;

  /* Initialize each DMA stream */

  for (i = 0; i < sizeof(g_chan) / sizeof(struct gpdma_ch_s); i++)
    {
      chan = &g_chan[i];

      /* Attach DMA interrupt vectors */

      irq_attach(chan->irq, gpdma_dmainterrupt, chan);

      /* Disable the DMA channel */

      gpdma_ch_disable(chan);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(chan->irq);
    }
}

/****************************************************************************
 * Name: stm32_dmachannel
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(enum gpdma_ttype_e type)
{
  /* Peripherals can be mapped to any number of different channels.
   *
   * P2M and M2P should be on channels with 8 byte FIFOs. GPDMA1/2 CH[0-3]
   *
   * Linear M2M should be on channels with 32 byte FIFOs. GPDMA1/2 CH[4-5]
   * Some peripherals (e.g. OCTOSPI) can also use these for burst transfers.
   *
   * 2D addressed transfers should be on GPDMA1/2 CH[6-7]
   *
   * Note: Currently, only P2M and M2P modes are supported and implemented.
   */

  DMA_HANDLE handle = NULL;
  irqstate_t flags;
  int i;

  /* Currently no support for M2M or 2D addressing modes.
   * TODO: Remove when support is added!
   */

  DEBUGASSERT(type != GPDMA_TTYPE_M2M_LINEAR);
  DEBUGASSERT(type != GPDMA_TTYPE_2D);

  flags = enter_critical_section();

  if (type == GPDMA_TTYPE_M2P || type == GPDMA_TTYPE_P2M)
    {
      for (i = 0; i < (sizeof(g_chan) / sizeof(struct gpdma_ch_s)); i++)
        {
          struct gpdma_ch_s *chan = &g_chan[i];

          if (chan->free)
            {
              chan->free = false;
              chan->type = type;
              handle = (DMA_HANDLE)chan;

#ifdef CONFIG_STM32_HAVE_GPDMA_SECURE
              /* Give the channel the security and privilege of the CPU,
               * while it is disabled.
               */

              modifyreg32(chan->base - CH_BASE_OFFSET(chan->channel) +
                          STM32_GPDMA_SECCFGR_OFFSET, 0, 1 << chan->channel);
              modifyreg32(chan->base - CH_BASE_OFFSET(chan->channel) +
                          STM32_GPDMA_PRIVCFGR_OFFSET, 0,
                          1 << chan->channel);
#endif
              break;
            }
        }
    }

  leave_critical_section(flags);

  if (handle == NULL)
    {
      /* Failed to allocate channel. */

      dmainfo("No available DMA chan for transfer type=%" PRIu8 "\n",
              type);
    }

  return handle;
}

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
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

void stm32_dmafree(DMA_HANDLE handle)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;

  DEBUGASSERT(handle != NULL);

  chan->free = true;
}

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

int stm32_dmasetup(DMA_HANDLE handle, struct stm32_gpdma_cfg_s *cfg)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Store the configuration so it can be referenced later by start. */

  chan->cfg = *cfg;
  chan->chain = false;

  /* Drivers using DMA should manage channel usage. If a DMA request is not
   * made on an error or an abort occurs, the driver should stop the DMA. If
   * it fails to do so, we can not just hang waiting on the HW that will not
   * change state.
   *
   * If at the end of waiting the HW is still not ready, there is a HW
   * or software usage problem.
   */

  gpdma_ch_disable(chan);

  /*  if ((gpdmach_getreg(chan, CH_CXCR_OFFSET) & GPDMA_CXCR_EN) != 0)
   *    {
   *      gpdma_ch_abort(chan);
   *    }
   */

  /* Clear any unhandled flags from previous transactions */

  gpdmach_putreg(chan, CH_CXFCR_OFFSET, ~0);

  if (cfg->mode & GPDMACFG_MODE_CIRC)
    {
      return gpdma_setup_circular(chan, cfg);
    }
  else
    {
      return gpdma_setup(chan, cfg);
    }
}

/****************************************************************************
 * Name: stm32_dmalli_init
 *
 * Description:
 *   Fill one LLI from a transfer configuration.
 *
 ****************************************************************************/

int stm32_dmalli_init(struct stm32_gpdma_lli_s *lli,
                      const struct stm32_gpdma_cfg_s *cfg,
                      const struct stm32_gpdma_lli_s *next)
{
  uint32_t tr1;
  uint32_t tr2;
  int ret;

  DEBUGASSERT(cfg != NULL);

  if (gpdma_check_lli_addr(lli) < 0)
    {
      return -EINVAL;
    }

  if (next != NULL &&
      (((uintptr_t)next & 3) != 0 || !GPDMA_SAME_SEGMENT(lli, next)))
    {
      return -EINVAL;
    }

  tr1 = cfg->tr1 | GPDMA_CXTR1_SECATTR;
  ret = gpdma_check_block(tr1, cfg->ntransfers, cfg->src_addr,
                          cfg->dest_addr);
  if (ret < 0)
    {
      return ret;
    }

  ret = gpdma_tr2(cfg, &tr2);
  if (ret < 0)
    {
      return ret;
    }

  lli->tr1 = tr1;
  lli->tr2 = tr2;
  lli->br1 = cfg->ntransfers;
  lli->sar = cfg->src_addr;
  lli->dar = cfg->dest_addr;
  lli->llr = next == NULL ? 0 :
             GPDMA_LLR_UPDATE_ALL | ((uintptr_t)next & GPDMA_CXLLR_LA_MASK);

  return OK;
}

/****************************************************************************
 * Name: stm32_dmasetup_lli
 *
 * Description:
 *   Configure the channel to run a chain of LLIs, starting with head.
 *
 ****************************************************************************/

int stm32_dmasetup_lli(DMA_HANDLE handle,
                       const struct stm32_gpdma_lli_s *head,
                       uint8_t priority, uint8_t mode)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;
  const struct stm32_gpdma_lli_s *lli = head;
  uintptr_t segment;
  size_t i;

  DEBUGASSERT(handle != NULL);

  if (gpdma_check_lli_addr(head) < 0)
    {
      return -EINVAL;
    }

  /* Walk the chain.  It ends at an LLI with LLR = 0, or it loops; a chain
   * longer than a segment can hold LLIs is a loop.
   */

  segment = (uintptr_t)head & GPDMA_SEGMENT_MASK;

  for (i = 0; i < GPDMA_LLI_MAX; i++)
    {
      if (gpdma_check_lli_addr(lli) < 0 ||
          gpdma_check_block(lli->tr1, lli->br1, lli->sar, lli->dar) < 0)
        {
          return -EINVAL;
        }

      if (lli->llr == 0)
        {
          break;
        }

      if ((lli->llr & ~GPDMA_CXLLR_LA_MASK) != GPDMA_LLR_UPDATE_ALL)
        {
          return -EINVAL;
        }

      lli = (const struct stm32_gpdma_lli_s *)
            (segment | (lli->llr & GPDMA_CXLLR_LA_MASK));
    }

  /* The chain is valid: program the channel with the head LLI */

  gpdma_ch_disable(chan);
  gpdmach_putreg(chan, CH_CXFCR_OFFSET, ~0);

  memset(&chan->cfg, 0, sizeof(chan->cfg));
  chan->cfg.priority = priority;
  chan->cfg.mode = mode & GPDMACFG_MODE_LAP1;
  chan->chain = true;

  gpdma_set_cr(chan, priority, mode);

  gpdmach_putreg(chan, CH_CXTR1_OFFSET,  head->tr1);
  gpdmach_putreg(chan, CH_CXTR2_OFFSET,  head->tr2);
  gpdmach_putreg(chan, CH_CXBR1_OFFSET,  head->br1);
  gpdmach_putreg(chan, CH_CXSAR_OFFSET,  head->sar);
  gpdmach_putreg(chan, CH_CXDAR_OFFSET,  head->dar);
  gpdmach_putreg(chan, CH_CXLBAR_OFFSET, segment);
  gpdmach_putreg(chan, CH_CXLLR_OFFSET,  head->llr);

  return OK;
}

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;
  uint32_t cr;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info. This will be invoked when the DMA completes */

  chan->callback = callback;
  chan->arg = arg;

  /* Activate channel by setting ENABLE bin in the GPDMA_CXCR register.
   * As soon as the channel is enabled, it can serve any DMA request from the
   * peripheral connected to the stream.
   */

  cr = gpdmach_getreg(chan, CH_CXCR_OFFSET);
  cr |= GPDMA_CXCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * mode, always interrupt on wrap and optionally interrupt at halfway.
   */

  if (chan->chain)
    {
      /* A chain reports TC (and HT, if asked) as set by TCEM in each LLI,
       * and any error, since an error disables the channel.
       */

      cr |= ((half ? GPDMA_CXCR_HTIE : 0) |
             GPDMA_CXCR_TCIE  |
             GPDMA_CXCR_DTEIE |
             GPDMA_CXCR_ULEIE |
             GPDMA_CXCR_USEIE);
    }
  else if (chan->cfg.mode & (GPDMACFG_MODE_CIRC))
    {
      /* In circular mode, when transfer completes it resets and starts
       * again. The transfer-complete interrupt is thus always enabled, and
       * the half-complete interrupt can be used to determine when the buffer
       * is half-full.
       */

      cr |= ((half ? GPDMA_CXCR_HTIE : 0) |
                          GPDMA_CXCR_TCIE |
                          GPDMA_CXCR_DTEIE);
    }
  else
    {
      /* Once half of the bytes are transferred, the half-transfer flag
       * (HTIF) is set and an interrupt is generated if the
       * Half-Transfer Interrupt Enable bit (HTIE) is set. At the end of the
       * transfer, the Transfer Complete Flag (TCIF) is set and an interrupt
       * is generated if the Transfer Complete Interrupt Enable bit (TCIE) is
       * set.
       */

      cr |= (half ? (GPDMA_CXCR_HTIE | GPDMA_CXCR_DTEIE) :
                    (GPDMA_CXCR_TCIE | GPDMA_CXCR_DTEIE));
    }

  gpdmach_putreg(chan, CH_CXCR_OFFSET, cr);
}

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA. After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;

  DEBUGASSERT(handle != NULL);

  gpdma_ch_disable(chan);
}

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Returns the number of data beats remaining to transfer in the current
 *   GPDMA block.  This reads the BNDT[15:0] field from the
 *   GPDMA_CxBR1 register, which indicates how many beats are left in the
 *   programmed transfer.
 *
 * Assumptions:
 *   - DMA handle was allocated by stm32_dmachannel().
 *   - The handle refers to a valid GPDMA channel.
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;
  uint32_t           br1  = getreg32(chan->base + CH_CXBR1_OFFSET);

  /* BNDT[15:0] = beats remaining in current block transfer */

  return (size_t)(br1 & GPDMA_CXBR1_BNDT_MASK);
}

#ifdef CONFIG_STM32_DMACAPABLE
/****************************************************************************
 * Name: stm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address. This depends on the internal connections in the ARM bus matrix
 *   of the processor. Note that this only applies to memory addresses, it
 *   will return false for any peripheral address.
 *
 * Input Parameters:
 *   cfg - DMA transfer configuration
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 * NOTE: No implementation yet.
 *
 ****************************************************************************/

bool stm32_dmacapable(DMA_HANDLE handle, stm32_gpdma_cfg_s *cfg)
{
# warning "stm32_dma_capable not implemented yet"
  return false;
}
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 * NOTE: No implementation yet.
 *
 ****************************************************************************/

static void stm32_dmadump(DMA_HANDLE handle, const char *msg)
{
#  warning "stm32_dma_dump not implemented yet!"
}
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
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

void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs)
{
  #warning "stm32_dmasample() not implemented yet!"
}
#endif
