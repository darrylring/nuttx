/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dac.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>
#include <nuttx/semaphore.h>
#include <nuttx/analog/dac.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_dac.h"
#include "stm32_rcc.h"
#include "stm32_dma.h"
#include "stm32_tim.h"
#include "stm32_tim_clk.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only DAC1 is supported, but it has two channels */

#if !defined(CONFIG_STM32_DAC1)
#  error "DAC1 must be enabled"
#endif

#if !defined(CONFIG_STM32_DAC1CH1) && !defined(CONFIG_STM32_DAC1CH2)
#  error "At least one DAC1 channel must be enabled"
#endif

/* DMA and timer trigger configuration
 *
 * A channel with DMA is paced by the TRGO output of a timer, which triggers
 * the DAC and makes it request the next sample from the GPDMA.  The timer
 * must be reserved for the DAC with CONFIG_STM32_TIMn_DAC.
 */

#if defined(CONFIG_STM32_DAC1CH1_DMA) || defined(CONFIG_STM32_DAC1CH2_DMA)
#  define HAVE_DMA 1

#  if !defined(CONFIG_STM32_DMA1) && !defined(CONFIG_STM32_DMA2)
#    error "DAC DMA requires CONFIG_STM32_DMA1 or CONFIG_STM32_DMA2"
#  endif
#endif

/* Timer trigger selection (TSELx).  The values for TIM1, TIM2, TIM5-TIM8 and
 * TIM15 are the same on all parts.  TSEL=3 is TIM4 on most parts and TIM3 on
 * the STM32H50x.
 */

#define DAC_TSEL_TIM1         1
#define DAC_TSEL_TIM2         2
#define DAC_TSEL_TIM5         4
#define DAC_TSEL_TIM6         5
#define DAC_TSEL_TIM7         6
#define DAC_TSEL_TIM8         7
#define DAC_TSEL_TIM15        8
#ifdef CONFIG_STM32_STM32H50XXX
#  define DAC_TSEL_TIM3       3
#else
#  define DAC_TSEL_TIM4       3
#endif

#ifdef CONFIG_STM32_STM32H50XXX
#  define DAC_TIMER_VALID(n)  ((n) == 1 || (n) == 2 || (n) == 3 || \
                               (n) == 6 || (n) == 7 || (n) == 15)
#else
#  define DAC_TIMER_VALID(n)  ((n) == 1 || (n) == 2 || (n) == 4 || \
                               (n) == 5 || (n) == 6 || (n) == 7 || \
                               (n) == 8 || (n) == 15)
#endif

/* Token pasting helpers.  The extra level lets the Kconfig timer number
 * expand before it is pasted.
 */

#define DAC_PASTE(a,n,b)      a##n##b
#define DAC_TSEL(n)           DAC_PASTE(DAC_TSEL_TIM,n,)
#define DAC_TIMBASE(n)        DAC_PASTE(STM32_TIM,n,_BASE)
#define DAC_TIMCLKIN(n)       DAC_PASTE(STM32_TIM,n,_CLKIN)
#define DAC_TIMEN_REG(n)      DAC_PASTE(STM32_RCC_TIM,n,_EN_REG)
#define DAC_TIMEN(n)          DAC_PASTE(STM32_RCC_TIM,n,_EN)
#define DAC_TIM_RESERVED(n)   DAC_PASTE(CONFIG_STM32_TIM,n,_DAC)
#define DAC_TIM_RESERVED_X(n) DAC_TIM_RESERVED(n)

#define DAC_TSEL_X(n)         DAC_TSEL(n)
#define DAC_TIMBASE_X(n)      DAC_TIMBASE(n)
#define DAC_TIMCLKIN_X(n)     DAC_TIMCLKIN(n)
#define DAC_TIMEN_REG_X(n)    DAC_TIMEN_REG(n)
#define DAC_TIMEN_X(n)        DAC_TIMEN(n)

/* The DAC channels are paced by timers 1, 2, 4, 5, 6, 7, 8 or 15 (3 instead
 * of 4 on the STM32H50x).  Check the configuration of each channel.
 */

#ifdef CONFIG_STM32_DAC1CH1_DMA
#  if !defined(CONFIG_STM32_DAC1CH1_TIMER) || \
      CONFIG_STM32_DAC1CH1_TIMER < 1
#    error "CONFIG_STM32_DAC1CH1_TIMER must select a timer"
#  endif
#  if !DAC_TIMER_VALID(CONFIG_STM32_DAC1CH1_TIMER)
#    error "CONFIG_STM32_DAC1CH1_TIMER is not a DAC trigger timer"
#  endif
#  if !DAC_TIM_RESERVED_X(CONFIG_STM32_DAC1CH1_TIMER)
#    error "The DAC1CH1 timer must be reserved with CONFIG_STM32_TIMn_DAC"
#  endif
#  if !defined(CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY) || \
      CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY < 1
#    error "CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY must be greater than 0"
#  endif
#  if CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE < 2 || \
      (CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE & 1) != 0 || \
      CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE > 32767
#    error "CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE must be even and <= 32766"
#  endif
#endif

#ifdef CONFIG_STM32_DAC1CH2_DMA
#  if !defined(CONFIG_STM32_DAC1CH2_TIMER) || \
      CONFIG_STM32_DAC1CH2_TIMER < 1
#    error "CONFIG_STM32_DAC1CH2_TIMER must select a timer"
#  endif
#  if !DAC_TIMER_VALID(CONFIG_STM32_DAC1CH2_TIMER)
#    error "CONFIG_STM32_DAC1CH2_TIMER is not a DAC trigger timer"
#  endif
#  if !DAC_TIM_RESERVED_X(CONFIG_STM32_DAC1CH2_TIMER)
#    error "The DAC1CH2 timer must be reserved with CONFIG_STM32_TIMn_DAC"
#  endif
#  if !defined(CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY) || \
      CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY < 1
#    error "CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY must be greater than 0"
#  endif
#  if CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE < 2 || \
      (CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE & 1) != 0 || \
      CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE > 32767
#    error "CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE must be even and <= 32766"
#  endif
#endif

/* GPDMA priority from the per-channel Kconfig choice */

#if defined(CONFIG_STM32_DAC1CH1_DMA_PRIORITY_LOW)
#  define DAC1CH1_DMA_PRI     GPDMACFG_PRIO_LL
#elif defined(CONFIG_STM32_DAC1CH1_DMA_PRIORITY_MEDIUM)
#  define DAC1CH1_DMA_PRI     GPDMACFG_PRIO_LM
#elif defined(CONFIG_STM32_DAC1CH1_DMA_PRIORITY_HIGH)
#  define DAC1CH1_DMA_PRI     GPMDACFG_PRIO_LH
#else
#  define DAC1CH1_DMA_PRI     GPDMACFG_PRIO_H
#endif

#if defined(CONFIG_STM32_DAC1CH2_DMA_PRIORITY_LOW)
#  define DAC1CH2_DMA_PRI     GPDMACFG_PRIO_LL
#elif defined(CONFIG_STM32_DAC1CH2_DMA_PRIORITY_MEDIUM)
#  define DAC1CH2_DMA_PRI     GPDMACFG_PRIO_LM
#elif defined(CONFIG_STM32_DAC1CH2_DMA_PRIORITY_HIGH)
#  define DAC1CH2_DMA_PRI     GPMDACFG_PRIO_LH
#else
#  define DAC1CH2_DMA_PRI     GPDMACFG_PRIO_H
#endif

/* Number of completed half-buffer events that can be queued.  Must be a
 * power of two.
 */

#define DAC_EVENTQ_SIZE       16

/* The holding register is written with 32-bit accesses, so each 16-bit
 * sample is zero extended by the DMA.  DHR12Rx only holds 12 bits.
 */

#define DAC_SAMPLE_MASK       0xfff

/* Helper macros for register bits */

#define DAC_CR_EN(ch)         ((ch) == 1 ? DAC_CR_EN1 : DAC_CR_EN2)
#define DAC_CR_TSEL_MASK(ch)  ((ch) == 1 ? DAC_CR_TSEL1_MASK : DAC_CR_TSEL2_MASK)
#define DAC_CR_TSEL_SHIFT(ch) ((ch) == 1 ? DAC_CR_TSEL1_SHIFT : DAC_CR_TSEL2_SHIFT)
#define DAC_CR_TEN(ch)        ((ch) == 1 ? DAC_CR_TEN1 : DAC_CR_TEN2)
#define DAC_CR_DMAEN(ch)      ((ch) == 1 ? DAC_CR_DMAEN1 : DAC_CR_DMAEN2)
#define DAC_SR_DMAUDR(ch)     ((ch) == 1 ? DAC_SR_DMAUDR1 : DAC_SR_DMAUDR2)
#define DAC_CR_WAVE_MASK(ch)  ((ch) == 1 ? DAC_CR_WAVE1_MASK : DAC_CR_WAVE2_MASK)
#define DAC_CR_MAMP_MASK(ch)  ((ch) == 1 ? DAC_CR_MAMP1_MASK : DAC_CR_MAMP2_MASK)
#define DAC_MCR_MODE_MASK(ch) ((ch) == 1 ? DAC_MCR_MODE1_MASK : DAC_MCR_MODE2_MASK)
#define DAC_MCR_MODE_EXT(ch)  ((ch) == 1 ? DAC_MCR_MODE1_NORM_EXT_BUF : \
                               DAC_MCR_MODE2_NORM_EXT_BUF)

/* The high-frequency interface mode must match the AHB clock frequency */

#if STM32_HCLK_FREQUENCY > 160000000
#  define DAC_MCR_HFSEL       DAC_MCR_HFSEL_AHB_160MHZ
#elif STM32_HCLK_FREQUENCY > 80000000
#  define DAC_MCR_HFSEL       DAC_MCR_HFSEL_AHB_80MHZ
#else
#  define DAC_MCR_HFSEL       DAC_MCR_HFSEL_DISABLED
#endif

/* Time to wait after enabling a channel before it accepts data (tWAKEUP is
 * at most a few microseconds; leave some margin).
 */

#define DAC_WAKEUP_USEC       15

/* Data holding register (12-bit, right aligned) for each channel */

#define DAC_DHR12R(ch)        ((ch) == 1 ? STM32_DAC1_DHR12R1 : STM32_DAC1_DHR12R2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the DAC block (DAC1) */

struct stm32_dac_s
{
  uint8_t init : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one DAC channel */

struct stm32_chan_s
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  const struct stm32_dac_ops_s *llops; /* Low-level DAC ops */
#endif
  uint8_t inuse  : 1;   /* True, the driver is in use */
  uint8_t ch;           /* Channel number (1 or 2) */
  uint32_t pin;         /* Pin configuration */
  uint32_t dro;         /* Data holding register address */
  uint32_t tsel;        /* CR trigger select value (not shifted) */
#ifdef HAVE_DMA
  uint8_t hasdma      : 1;  /* True, this channel is paced by DMA */
  uint8_t dma_active  : 1;  /* True, a DMA transfer is running */
  uint8_t halfint     : 1;  /* True, half-transfer events are queued */
  uint8_t dma_pri;          /* GPDMA priority */
  uint8_t dma_req;          /* GPDMA request number */
  uint16_t buffer_len;      /* DMA buffer length in samples */
  uint16_t *dmabuffer;      /* DMA transfer buffer */
  uint32_t tbase;           /* Timer base address */
  uint32_t tclkin;          /* Timer input clock frequency */
  uint32_t tfrequency;      /* Desired timer (sample) frequency */
  uint32_t ten_reg;         /* RCC enable register of the timer */
  uint32_t ten_bit;         /* RCC enable bit of the timer */
  DMA_HANDLE dma;           /* Allocated DMA channel */
  int result;               /* Last DMA error, OK if none */
  sem_t dma_halfsem;        /* Counts queued half-buffer events */

  /* Ring of completed half-buffer indices, filled by the DMA interrupt */

  uint8_t dma_q[DAC_EVENTQ_SIZE];
  uint8_t dma_q_head;       /* Next slot to fill */
  uint8_t dma_q_tail;       /* Next slot to read */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stm32_dac_dumpregs(struct stm32_chan_s *chan);
static void stm32_dac_enable(struct stm32_chan_s *chan);

static void stm32_dac_reset(struct dac_dev_s *dev);
static int  stm32_dac_setup(struct dac_dev_s *dev);
static void stm32_dac_shutdown(struct dac_dev_s *dev);
static void stm32_dac_txint(struct dac_dev_s *dev, bool enable);
static int  stm32_dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int  stm32_dac_ioctl(struct dac_dev_s *dev, int cmd,
                            unsigned long arg);

static int  stm32_dac_chaninit(struct stm32_chan_s *chan);
static void stm32_dac_blockinit(void);

#ifdef HAVE_DMA
static void stm32_dac_dmacallback(DMA_HANDLE handle, uint8_t status,
                                  void *arg);
static int  stm32_dac_timinit(struct stm32_chan_s *chan);
static void stm32_dac_dma_start(struct stm32_chan_s *chan, bool halfint);
static void stm32_dac_dma_stop(struct stm32_chan_s *chan);
#endif

#ifdef CONFIG_STM32_DAC_LL_OPS
static void stm32_dac_llops_enable(struct stm32_dac_dev_s *dev,
                                   bool enabled);
static void stm32_dac_llops_writedro(struct stm32_dac_dev_s *dev,
                                     uint16_t data);
static void stm32_dac_llops_dumpregs(struct stm32_dac_dev_s *dev);
#ifdef HAVE_DMA
static void stm32_dac_llops_startdma(struct stm32_dac_dev_s *dev);
static void stm32_dac_llops_stopdma(struct stm32_dac_dev_s *dev);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = stm32_dac_reset,
  .ao_setup    = stm32_dac_setup,
  .ao_shutdown = stm32_dac_shutdown,
  .ao_txint    = stm32_dac_txint,
  .ao_send     = stm32_dac_send,
  .ao_ioctl    = stm32_dac_ioctl,
};

#ifdef CONFIG_STM32_DAC_LL_OPS
static const struct stm32_dac_ops_s g_dac_llops =
{
  .enable        = stm32_dac_llops_enable,
  .write_dro     = stm32_dac_llops_writedro,
#ifdef HAVE_DMA
  .start_dma     = stm32_dac_llops_startdma,
  .stop_dma      = stm32_dac_llops_stopdma,
#endif
  .dump_regs     = stm32_dac_llops_dumpregs
};
#endif

#ifdef CONFIG_STM32_DAC1CH1
/* DAC1 channel 1 */

#ifdef CONFIG_STM32_DAC1CH1_DMA
static uint16_t g_dac1ch1_dmabuffer[CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE]
                aligned_data(4);
#endif

static struct stm32_chan_s g_dac1ch1priv =
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  .llops      = &g_dac_llops,
#endif
  .ch         = 1,
  .pin        = GPIO_DAC1_OUT1_0,
  .dro        = DAC_DHR12R(1),
#ifdef CONFIG_STM32_DAC1CH1_DMA
  .tsel       = DAC_TSEL_X(CONFIG_STM32_DAC1CH1_TIMER),
  .hasdma     = 1,
  .dma_pri    = DAC1CH1_DMA_PRI,
  .dma_req    = GPDMA_REQ_DAC1_CH1,
  .buffer_len = CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE,
  .dmabuffer  = g_dac1ch1_dmabuffer,
  .tbase      = DAC_TIMBASE_X(CONFIG_STM32_DAC1CH1_TIMER),
  .tclkin     = DAC_TIMCLKIN_X(CONFIG_STM32_DAC1CH1_TIMER),
  .tfrequency = CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY,
  .ten_reg    = DAC_TIMEN_REG_X(CONFIG_STM32_DAC1CH1_TIMER),
  .ten_bit    = DAC_TIMEN_X(CONFIG_STM32_DAC1CH1_TIMER),
#endif
};

static struct dac_dev_s g_dac1ch1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch1priv,
};
#endif /* CONFIG_STM32_DAC1CH1 */

#ifdef CONFIG_STM32_DAC1CH2
/* DAC1 channel 2 */

#ifdef CONFIG_STM32_DAC1CH2_DMA
static uint16_t g_dac1ch2_dmabuffer[CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE]
                aligned_data(4);
#endif

static struct stm32_chan_s g_dac1ch2priv =
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  .llops      = &g_dac_llops,
#endif
  .ch         = 2,
  .pin        = GPIO_DAC1_OUT2_0,
  .dro        = DAC_DHR12R(2),
#ifdef CONFIG_STM32_DAC1CH2_DMA
  .tsel       = DAC_TSEL_X(CONFIG_STM32_DAC1CH2_TIMER),
  .hasdma     = 1,
  .dma_pri    = DAC1CH2_DMA_PRI,
  .dma_req    = GPDMA_REQ_DAC1_CH2,
  .buffer_len = CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE,
  .dmabuffer  = g_dac1ch2_dmabuffer,
  .tbase      = DAC_TIMBASE_X(CONFIG_STM32_DAC1CH2_TIMER),
  .tclkin     = DAC_TIMCLKIN_X(CONFIG_STM32_DAC1CH2_TIMER),
  .tfrequency = CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY,
  .ten_reg    = DAC_TIMEN_REG_X(CONFIG_STM32_DAC1CH2_TIMER),
  .ten_bit    = DAC_TIMEN_X(CONFIG_STM32_DAC1CH2_TIMER),
#endif
};

static struct dac_dev_s g_dac1ch2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch2priv,
};
#endif /* CONFIG_STM32_DAC1CH2 */

static struct stm32_dac_s g_dacblock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_dumpregs
 ****************************************************************************/

static void stm32_dac_dumpregs(struct stm32_chan_s *chan)
{
  UNUSED(chan);

  ainfo("CR:   0x%08" PRIx32 "  SR:  0x%08" PRIx32 "  MCR: 0x%08" PRIx32
        "\n",
        getreg32(STM32_DAC1_CR), getreg32(STM32_DAC1_SR),
        getreg32(STM32_DAC1_MCR));
  ainfo("DOR1: 0x%08" PRIx32 "  DOR2: 0x%08" PRIx32 "\n",
        getreg32(STM32_DAC1_DOR1), getreg32(STM32_DAC1_DOR2));
}

/****************************************************************************
 * Name: stm32_dac_enable
 *
 * Description:
 *   Enable a channel and wait for it to wake up.  Data written to the
 *   holding register while the channel is still waking up can be lost, so
 *   this must complete before any data is written.
 *
 ****************************************************************************/

static void stm32_dac_enable(struct stm32_chan_s *chan)
{
  if ((getreg32(STM32_DAC1_CR) & DAC_CR_EN(chan->ch)) == 0)
    {
      modifyreg32(STM32_DAC1_CR, 0, DAC_CR_EN(chan->ch));
      up_udelay(DAC_WAKEUP_USEC);
    }
}

/****************************************************************************
 * Name: stm32_dac_reset
 ****************************************************************************/

static void stm32_dac_reset(struct dac_dev_s *dev)
{
  struct stm32_chan_s *chan = dev->ad_priv;

  /* Disable the channel and clear its trigger, waveform and mode setup */

  modifyreg32(STM32_DAC1_CR, DAC_CR_EN(chan->ch) |
              DAC_CR_TSEL_MASK(chan->ch) | DAC_CR_WAVE_MASK(chan->ch) |
              DAC_CR_MAMP_MASK(chan->ch), 0);
  modifyreg32(STM32_DAC1_MCR, DAC_MCR_MODE_MASK(chan->ch), 0);
}

/****************************************************************************
 * Name: stm32_dac_setup
 ****************************************************************************/

static int stm32_dac_setup(struct dac_dev_s *dev)
{
  struct stm32_chan_s *chan = dev->ad_priv;
  int ret;

  /* Configure the channel if not already configured */

  if (!chan->inuse)
    {
      ret = stm32_dac_chaninit(chan);
      if (ret < 0)
        {
          aerr("ERROR: DAC channel init failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_dac_shutdown
 ****************************************************************************/

static void stm32_dac_shutdown(struct dac_dev_s *dev)
{
  /* Leave the channel enabled so that the last value written keeps being
   * driven after the device is closed.  Applications such as the dac
   * example open, write and close the device for every update, and
   * disabling the channel here would put the output back into high
   * impedance.  stm32_dac_reset() is what turns the channel off.
   */
}

/****************************************************************************
 * Name: stm32_dac_txint
 ****************************************************************************/

static void stm32_dac_txint(struct dac_dev_s *dev, bool enable)
{
  /* No interrupt support for DAC */
}

/****************************************************************************
 * Name: stm32_dac_send
 ****************************************************************************/

static int stm32_dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
  struct stm32_chan_s *chan = dev->ad_priv;

#ifdef HAVE_DMA
  /* The holding register belongs to the DMA while it is running.  Consume
   * the message so that the FIFO does not stall.
   */

  if (chan->dma_active)
    {
      ainfo("DMA is running, dropping message\n");
      dac_txdone(dev);
      return OK;
    }
#endif

  /* The trigger is disabled, so the value is transferred to the output
   * as soon as it is written to the holding register.  The register must
   * be written with a 32-bit access; a halfword access faults on the H5.
   */

  stm32_dac_enable(chan);
  putreg32(msg->am_data & DAC_SAMPLE_MASK, chan->dro);
  dac_txdone(dev);

  return OK;
}

/****************************************************************************
 * Name: stm32_dac_ioctl
 ****************************************************************************/

static int stm32_dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
#ifdef HAVE_DMA
  struct stm32_chan_s *chan = dev->ad_priv;
#endif
  int ret = -ENOTTY;

  switch (cmd)
    {
#ifdef HAVE_DMA
      /* Copy a full buffer of samples to the DMA buffer */

      case ANIOC_DAC_DMABUFF_INIT:
        {
          const uint16_t *buffer = (const uint16_t *)arg;
          int i;

          if (!chan->hasdma)
            {
              ret = -ENOTTY;
              break;
            }

          if (buffer == NULL)
            {
              ret = -EINVAL;
              break;
            }

          for (i = 0; i < chan->buffer_len; i++)
            {
              chan->dmabuffer[i] = buffer[i] & DAC_SAMPLE_MASK;
            }

          up_clean_dcache((uintptr_t)chan->dmabuffer,
                          (uintptr_t)chan->dmabuffer +
                          chan->buffer_len * sizeof(uint16_t));
          ret = OK;
        }
        break;

      /* Start the circular DMA transfer */

      case ANIOC_DAC_DMA_START:
        {
          struct dac_dma_start_s *req = (struct dac_dma_start_s *)arg;

          if (!chan->hasdma)
            {
              ret = -ENOTTY;
              break;
            }

          if (req == NULL)
            {
              ret = -EINVAL;
              break;
            }

          stm32_dac_dma_start(chan, req->halfint != 0);
          ret = chan->dma_active ? OK : -EIO;
        }
        break;

      /* Stop the DMA transfer */

      case ANIOC_DAC_DMA_STOP:
        if (!chan->hasdma)
          {
            ret = -ENOTTY;
            break;
          }

        stm32_dac_dma_stop(chan);
        ret = OK;
        break;

      /* Wait for the next completed half of the buffer */

      case ANIOC_DAC_DMA_GET_EVENT:
        {
          struct dac_dma_event_s *req = (struct dac_dma_event_s *)arg;
          irqstate_t flags;

          if (!chan->hasdma)
            {
              ret = -ENOTTY;
              break;
            }

          if (req == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = nxsem_wait(&chan->dma_halfsem);
          if (ret < 0)
            {
              break;
            }

          flags = enter_critical_section();
          if (chan->dma_q_head != chan->dma_q_tail)
            {
              req->half = chan->dma_q[chan->dma_q_tail];
              chan->dma_q_tail = (chan->dma_q_tail + 1) &
                                 (DAC_EVENTQ_SIZE - 1);
              ret = OK;
            }
          else
            {
              /* Woken up without an event: the transfer failed or was
               * stopped.
               */

              ret = -ECANCELED;
            }

          /* A DMA error takes precedence */

          if (chan->result != OK)
            {
              ret = chan->result;
            }

          leave_critical_section(flags);
        }
        break;

      /* Refill one half of the DMA buffer */

      case ANIOC_DAC_DMAHBUF_WRITE:
        {
          struct dac_dma_event_s *req = (struct dac_dma_event_s *)arg;
          uint16_t half_len;
          uint16_t *dst;
          int i;

          if (!chan->hasdma)
            {
              ret = -ENOTTY;
              break;
            }

          if (req == NULL || req->buffer == NULL ||
              (req->half != 0 && req->half != 1))
            {
              ret = -EINVAL;
              break;
            }

          half_len = chan->buffer_len / 2;
          dst      = chan->dmabuffer + req->half * half_len;

          for (i = 0; i < half_len; i++)
            {
              dst[i] = req->buffer[i] & DAC_SAMPLE_MASK;
            }

          up_clean_dcache((uintptr_t)dst,
                          (uintptr_t)dst + half_len * sizeof(uint16_t));
          ret = OK;
        }
        break;
#endif /* HAVE_DMA */

      case ANIOC_DAC_INFO:
        {
          struct dac_info_s *info = (struct dac_info_s *)arg;

          if (info == NULL)
            {
              ret = -EINVAL;
              break;
            }

          info->sample_bits = 12;
#ifdef HAVE_DMA
          if (chan->hasdma)
            {
              info->dma_enabled         = chan->dma_active;
              info->halfint_enabled     = chan->halfint;
              info->dma_buffer_size     = chan->buffer_len;
              info->dma_timer_frequency = chan->tfrequency;
            }
          else
#endif
            {
              info->dma_enabled         = false;
              info->halfint_enabled     = 0;
              info->dma_buffer_size     = 0;
              info->dma_timer_frequency = 0;
            }

          ret = OK;
        }
        break;

      default:
        break;
    }

  return ret;
}

#ifdef HAVE_DMA

/****************************************************************************
 * Name: stm32_dac_queue_half
 *
 * Description:
 *   Queue a completed half-buffer event and wake up a waiter.  Called from
 *   the DMA interrupt.  The event is dropped if the queue is full.
 *
 ****************************************************************************/

static void stm32_dac_queue_half(struct stm32_chan_s *chan, uint8_t half)
{
  uint8_t next = (chan->dma_q_head + 1) & (DAC_EVENTQ_SIZE - 1);

  if (next != chan->dma_q_tail)
    {
      chan->dma_q[chan->dma_q_head] = half;
      chan->dma_q_head = next;
      nxsem_post(&chan->dma_halfsem);
    }
}

/****************************************************************************
 * Name: stm32_dac_dmacallback
 *
 * Description:
 *   DMA interrupt callback.  The half-transfer event means that the first
 *   half of the buffer has been output and may be refilled; the transfer
 *   complete event means the same for the second half.  Events are
 *   queued for ANIOC_DAC_DMA_GET_EVENT.
 *
 ****************************************************************************/

static void stm32_dac_dmacallback(DMA_HANDLE handle, uint8_t status,
                                  void *arg)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)arg;

  DEBUGASSERT(chan != NULL);

  if ((status & DMA_STATUS_FATAL) != 0)
    {
      /* Wake up a waiter, which sees the error */

      chan->result = -EIO;
      nxsem_post(&chan->dma_halfsem);
      return;
    }

  /* The half-transfer interrupt is only enabled when requested.  The
   * transfer complete event is always reported.
   */

  if ((status & DMA_STATUS_HTF) != 0 && chan->halfint)
    {
      stm32_dac_queue_half(chan, 0);
    }

  if ((status & DMA_STATUS_TCF) != 0)
    {
      stm32_dac_queue_half(chan, 1);
    }
}

/****************************************************************************
 * Name: stm32_dac_timinit
 *
 * Description:
 *   Configure the timer that paces the DAC.  The timer update event is
 *   routed to TRGO, which triggers the conversion.  The timer is left
 *   stopped.
 *
 ****************************************************************************/

static int stm32_dac_timinit(struct stm32_chan_s *chan)
{
  uint32_t div;
  uint32_t prescaler;
  uint32_t reload;

  if (chan->tfrequency == 0 || chan->tfrequency > chan->tclkin)
    {
      aerr("ERROR: Invalid timer frequency %" PRIu32 " Hz\n",
           chan->tfrequency);
      return -EINVAL;
    }

  modifyreg32(chan->ten_reg, 0, chan->ten_bit);

  /* The timer divides its input clock by (PSC + 1) * (ARR + 1), where both
   * factors are limited to 16 bits.
   */

  div       = (chan->tclkin + chan->tfrequency / 2) / chan->tfrequency;
  prescaler = (div + 65535) / 65536;
  if (prescaler < 1)
    {
      prescaler = 1;
    }

  reload = (div + prescaler / 2) / prescaler;
  if (reload < 1)
    {
      reload = 1;
    }
  else if (reload > 65536)
    {
      reload = 65536;
    }

  ainfo("timer clk=%" PRIu32 " freq=%" PRIu32 " psc=%" PRIu32
        " arr=%" PRIu32 "\n", chan->tclkin, chan->tfrequency,
        prescaler - 1, reload - 1);

  putreg32(0, chan->tbase + STM32_GTIM_CR1_OFFSET);
  putreg32(prescaler - 1, chan->tbase + STM32_GTIM_PSC_OFFSET);
  putreg32(reload - 1, chan->tbase + STM32_GTIM_ARR_OFFSET);

  modifyreg32(chan->tbase + STM32_GTIM_CR2_OFFSET, GTIM_CR2_MMS_MASK,
              GTIM_CR2_MMS_UPDATE);
  modifyreg32(chan->tbase + STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_ARPE);

  /* Load the prescaler and the reload value */

  putreg32(GTIM_EGR_UG, chan->tbase + STM32_GTIM_EGR_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: stm32_dac_dma_start
 *
 * Description:
 *   Start the circular transfer of the DMA buffer to the DAC, paced by the
 *   timer.
 *
 ****************************************************************************/

static void stm32_dac_dma_start(struct stm32_chan_s *chan, bool halfint)
{
  struct stm32_gpdma_cfg_s cfg;
  irqstate_t flags;

  if (chan->dma_active)
    {
      return;
    }

  /* Each 16-bit sample is written to the holding register with a 32-bit
   * access: the source data width is a half-word and the destination data
   * width is a word, so the GPDMA zero extends the sample.
   */

  cfg.src_addr   = (uintptr_t)chan->dmabuffer;
  cfg.dest_addr  = chan->dro;
  cfg.request    = GPDMA_CXTR2_DREQ | GPDMA_CXTR2_REQSEL(chan->dma_req);
  cfg.ntransfers = chan->buffer_len * sizeof(uint16_t);
  cfg.priority   = chan->dma_pri;
  cfg.mode       = GPDMACFG_MODE_CIRC;
  cfg.tr1        = GPDMA_CXTR1_SDW_LOG2_HW | GPDMA_CXTR1_SINC |
                   GPDMA_CXTR1_DDW_LOG2_WORD;

  flags = enter_critical_section();

  /* Discard events and errors of a previous transfer */

  chan->result     = OK;
  chan->halfint    = halfint;
  chan->dma_q_head = 0;
  chan->dma_q_tail = 0;
  while (nxsem_trywait(&chan->dma_halfsem) == OK)
    {
    }

  up_clean_dcache((uintptr_t)chan->dmabuffer,
                  (uintptr_t)chan->dmabuffer +
                  chan->buffer_len * sizeof(uint16_t));

  stm32_dmasetup(chan->dma, &cfg);
  stm32_dmastart(chan->dma, stm32_dac_dmacallback, chan, halfint);

  /* DMA requests stop after an underrun until the flag is cleared */

  putreg32(DAC_SR_DMAUDR(chan->ch), STM32_DAC1_SR);

  /* Restart the timer from zero before the trigger is enabled, so that
   * the first period has the full length.  The update event only toggles
   * TRGO and is ignored by the DAC while TEN is clear.
   */

  putreg32(GTIM_EGR_UG, chan->tbase + STM32_GTIM_EGR_OFFSET);

  stm32_dac_enable(chan);
  modifyreg32(STM32_DAC1_CR, 0, DAC_CR_TEN(chan->ch) |
              DAC_CR_DMAEN(chan->ch));
  modifyreg32(chan->tbase + STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);

  chan->dma_active = 1;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_dac_dma_stop
 *
 * Description:
 *   Stop the DMA transfer.  The channel stays enabled and keeps driving the
 *   last value.
 *
 ****************************************************************************/

static void stm32_dac_dma_stop(struct stm32_chan_s *chan)
{
  irqstate_t flags;
  int sval;

  if (!chan->dma_active)
    {
      return;
    }

  flags = enter_critical_section();

  modifyreg32(chan->tbase + STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
  modifyreg32(STM32_DAC1_CR, DAC_CR_TEN(chan->ch) | DAC_CR_DMAEN(chan->ch),
              0);
  stm32_dmastop(chan->dma);

  chan->dma_active = 0;

  /* Release a thread waiting for an event */

  if (nxsem_get_value(&chan->dma_halfsem, &sval) == OK && sval < 0)
    {
      nxsem_post(&chan->dma_halfsem);
    }

  leave_critical_section(flags);
}

#endif /* HAVE_DMA */

/****************************************************************************
 * Name: stm32_dac_chaninit
 ****************************************************************************/

static int stm32_dac_chaninit(struct stm32_chan_s *chan)
{
  if (chan->inuse)
    {
      return -EBUSY;
    }

  stm32_configgpio(chan->pin);

  /* Disable the channel before configuration.  Leave the trigger disabled
   * (TEN=0) with no wave generation so that writes to the holding register
   * update the output directly.
   */

  modifyreg32(STM32_DAC1_CR, DAC_CR_EN(chan->ch) |
              DAC_CR_TSEL_MASK(chan->ch) | DAC_CR_WAVE_MASK(chan->ch) |
              DAC_CR_MAMP_MASK(chan->ch), 0);

  /* Normal mode, output to the external pin with the buffer enabled.  The
   * high-frequency interface mode is shared by both channels.
   */

  modifyreg32(STM32_DAC1_MCR, DAC_MCR_MODE_MASK(chan->ch) |
              DAC_MCR_HFSEL_MASK, DAC_MCR_MODE_EXT(chan->ch) |
              DAC_MCR_HFSEL);

#ifdef HAVE_DMA
  if (chan->hasdma)
    {
      int ret;

      /* Select the trigger.  The trigger itself stays disabled (TEN=0) and
       * writes go straight to the output until the DMA transfer is started.
       */

      modifyreg32(STM32_DAC1_CR, 0,
                  chan->tsel << DAC_CR_TSEL_SHIFT(chan->ch));

      chan->dma = stm32_dmachannel(GPDMA_TTYPE_M2P);
      if (chan->dma == NULL)
        {
          aerr("ERROR: No DMA channel available for DAC channel %d\n",
               chan->ch);
          return -EBUSY;
        }

      ret = stm32_dac_timinit(chan);
      if (ret < 0)
        {
          stm32_dmafree(chan->dma);
          chan->dma = NULL;
          return ret;
        }

      nxsem_init(&chan->dma_halfsem, 0, 0);
      nxsem_set_protocol(&chan->dma_halfsem, SEM_PRIO_NONE);
    }
#endif

  /* Enable the channel now so that it has woken up by the first write */

  stm32_dac_enable(chan);

  chan->inuse = 1;
  return OK;
}

/****************************************************************************
 * Name: stm32_dac_blockinit
 ****************************************************************************/

static void stm32_dac_blockinit(void)
{
  irqstate_t flags;

  if (g_dacblock.init)
    {
      return;
    }

  flags = enter_critical_section();

  /* Enable the DAC1 clock (AHB2) and pulse its reset */

  modifyreg32(STM32_RCC_AHB2ENR, 0, RCC_AHB2ENR_DAC1EN);
  modifyreg32(STM32_RCC_AHB2RSTR, 0, RCC_AHB2RSTR_DACRST);
  modifyreg32(STM32_RCC_AHB2RSTR, RCC_AHB2RSTR_DACRST, 0);

  g_dacblock.init = 1;
  leave_critical_section(flags);
}

#ifdef CONFIG_STM32_DAC_LL_OPS

/****************************************************************************
 * Name: stm32_dac_llops_enable
 ****************************************************************************/

static void stm32_dac_llops_enable(struct stm32_dac_dev_s *dev, bool enabled)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)dev;

  if (enabled)
    {
      stm32_dac_enable(chan);
    }
  else
    {
      modifyreg32(STM32_DAC1_CR, DAC_CR_EN(chan->ch), 0);
    }
}

/****************************************************************************
 * Name: stm32_dac_llops_writedro
 ****************************************************************************/

static void stm32_dac_llops_writedro(struct stm32_dac_dev_s *dev,
                                     uint16_t data)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)dev;

  putreg32(data & DAC_SAMPLE_MASK, chan->dro);
}

#ifdef HAVE_DMA
/****************************************************************************
 * Name: stm32_dac_llops_startdma
 ****************************************************************************/

static void stm32_dac_llops_startdma(struct stm32_dac_dev_s *dev)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)dev;

  if (chan->hasdma && chan->inuse)
    {
      stm32_dac_dma_start(chan, false);
    }
}

/****************************************************************************
 * Name: stm32_dac_llops_stopdma
 ****************************************************************************/

static void stm32_dac_llops_stopdma(struct stm32_dac_dev_s *dev)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)dev;

  if (chan->hasdma)
    {
      stm32_dac_dma_stop(chan);
    }
}
#endif

/****************************************************************************
 * Name: stm32_dac_llops_dumpregs
 ****************************************************************************/

static void stm32_dac_llops_dumpregs(struct stm32_dac_dev_s *dev)
{
  stm32_dac_dumpregs((struct stm32_chan_s *)dev);
}

#endif /* CONFIG_STM32_DAC_LL_OPS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.  The interface number corresponds to the channel:
 *   0 = DAC1 channel 1, 1 = DAC1 channel 2.
 *
 ****************************************************************************/

struct dac_dev_s *stm32_dacinitialize(int intf)
{
  struct dac_dev_s *dev;

  switch (intf)
    {
#ifdef CONFIG_STM32_DAC1CH1
      case 0:
        dev = &g_dac1ch1dev;
        break;
#endif
#ifdef CONFIG_STM32_DAC1CH2
      case 1:
        dev = &g_dac1ch2dev;
        break;
#endif
      default:
        aerr("ERROR: Invalid DAC interface: %d\n", intf);
        return NULL;
    }

  stm32_dac_blockinit();

  return dev;
}

#endif /* CONFIG_DAC */
