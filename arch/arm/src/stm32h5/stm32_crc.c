/****************************************************************************
 * arch/arm/src/stm32h5/stm32_crc.c
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

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/crc/crc.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "stm32_crc.h"

#ifdef CONFIG_STM32_CRC_DMA
#  include "stm32_dma.h"
#endif

#include "hardware/stm32_crc.h"
#include "hardware/stm32h5xxx_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CR.RESET self-clears within a few AHB clock cycles; this bounds how
 * long we spin waiting for that, rather than hanging forever if the
 * engine is ever unexpectedly wedged.
 */

#define STM32_CRC_RESET_TIMEOUT  1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_crc_dev_s
{
  struct crc_lowerhalf_s lower;

#ifdef CONFIG_STM32_CRC_DMA
  DMA_HANDLE dma;         /* Cached M2P channel, held for the driver's
                            * lifetime once acquired */
  sem_t      dmadone;     /* Posted by stm32_crc_dmacallback() */
  uint8_t    dmaresult;   /* DMA_STATUS_* bits from the last transfer,
                            * OR'ed with 0x80 to distinguish "not yet
                            * posted" from a legitimately all-zero
                            * status */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  stm32_crc_setup(FAR struct crc_lowerhalf_s *lower);
static void stm32_crc_shutdown(FAR struct crc_lowerhalf_s *lower);
static int  stm32_crc_configure(FAR struct crc_lowerhalf_s *lower,
                                FAR const struct crc_config_s *config);
static int  stm32_crc_checkpoint_load(FAR struct crc_lowerhalf_s *lower,
                                      uint32_t state);
static int  stm32_crc_write_cpu(FAR struct crc_lowerhalf_s *lower,
                                FAR const uint8_t *buffer, size_t len,
                                FAR uint32_t *result);
#ifdef CONFIG_STM32_CRC_DMA
static int  stm32_crc_write_dma(FAR struct crc_lowerhalf_s *lower,
                                FAR const uint8_t *buffer, size_t len,
                                FAR uint32_t *result);
#endif
static int  stm32_crc_write(FAR struct crc_lowerhalf_s *lower,
                            FAR const uint8_t *buffer, size_t len,
                            FAR uint32_t *result);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct crc_ops_s g_stm32_crc_ops =
{
  .setup           = stm32_crc_setup,
  .shutdown        = stm32_crc_shutdown,
  .configure       = stm32_crc_configure,
  .checkpoint_load = stm32_crc_checkpoint_load,
  .write           = stm32_crc_write,
};

/* There is exactly one physical CRC engine. */

static struct stm32_crc_dev_s g_stm32_crc_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_crc_wait_reset
 *
 * Description:
 *   Spin until CR.RESET self-clears, indicating that DR has been
 *   reloaded from INIT.
 *
 ****************************************************************************/

static int stm32_crc_wait_reset(void)
{
  int i;

  for (i = 0; i < STM32_CRC_RESET_TIMEOUT; i++)
    {
      if ((getreg32(STM32_CRC_CR) & CRC_CR_RESET) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

static int stm32_crc_setup(FAR struct crc_lowerhalf_s *lower)
{
#ifdef CONFIG_STM32_CRC_DMA
  FAR struct stm32_crc_dev_s *priv =
    (FAR struct stm32_crc_dev_s *)lower->cl_priv;
#endif
  irqstate_t flags;
  uint32_t regval;

  /* Pulse the peripheral reset to bring the engine to a known state.
   * Its clock is already enabled unconditionally at boot whenever
   * CONFIG_STM32_CRC is set (see stm32h5xx_rcc.c).
   */

  flags   = enter_critical_section();
  regval  = getreg32(STM32_RCC_AHB1RSTR);
  regval |= RCC_AHB1RSTR_CRCRST;
  putreg32(regval, STM32_RCC_AHB1RSTR);
  regval &= ~RCC_AHB1RSTR_CRCRST;
  putreg32(regval, STM32_RCC_AHB1RSTR);
  leave_critical_section(flags);

#ifdef CONFIG_STM32_CRC_DMA
  /* Acquire the DMA channel once and hold it for the driver's lifetime
   * rather than per-transfer, since re-acquiring would race against
   * other DMA users for a scarce channel.
   */

  if (priv->dma == NULL)
    {
      priv->dma = stm32_dmachannel(GPDMA_TTYPE_M2P);
      if (priv->dma == NULL)
        {
          _err("ERROR: Failed to allocate CRC DMA channel, "
               "falling back to non-DMA writes\n");
        }
    }
#endif

  return OK;
}

static void stm32_crc_shutdown(FAR struct crc_lowerhalf_s *lower)
{
  /* Nothing to do: the DMA channel (if any) and the peripheral clock
   * remain reserved for the lifetime of this lower half.
   */
}

static int stm32_crc_configure(FAR struct crc_lowerhalf_s *lower,
                                FAR const struct crc_config_s *config)
{
  uint32_t regval;
  uint32_t polysize;

  switch (config->width)
    {
      case 32:
        polysize = CRC_CR_POLYSIZE_32;
        break;

      case 16:
        polysize = CRC_CR_POLYSIZE_16;
        break;

      case 8:
        polysize = CRC_CR_POLYSIZE_8;
        break;

      case 7:
        polysize = CRC_CR_POLYSIZE_7;
        break;

      default:
        return -EINVAL;
    }

  regval  = getreg32(STM32_CRC_CR);
  regval &= ~(CRC_CR_POLYSIZE_MASK | CRC_CR_REVIN_MASK | CRC_CR_REVOUT);
  regval |= polysize;

  if (config->refin)
    {
      regval |= CRC_CR_REVIN_BYTE;
    }

  /* CRC_CR_REVOUT is intentionally never set.  It only reverses bits as
   * they are read out of CRC_DR -- it does not affect the engine's
   * internal running state.  If a checkpoint read taken with REV_OUT
   * set were written back into CRC_INIT to resume a later chunk, the
   * internal state would be silently corrupted.  Any output reflection
   * requested via 'refout' is applied by the upper half in software,
   * once, only on the final result.
   */

  putreg32(regval, STM32_CRC_CR);
  putreg32(config->poly, STM32_CRC_POL);

  return OK;
}

static int stm32_crc_checkpoint_load(FAR struct crc_lowerhalf_s *lower,
                                      uint32_t state)
{
  putreg32(state, STM32_CRC_INIT);
  modifyreg32(STM32_CRC_CR, 0, CRC_CR_RESET);
  return stm32_crc_wait_reset();
}

static int stm32_crc_write_cpu(FAR struct crc_lowerhalf_s *lower,
                               FAR const uint8_t *buffer, size_t len,
                               FAR uint32_t *result)
{
  uint32_t regval;

  while (len >= sizeof(uint32_t))
    {
      regval = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
               ((uint32_t)buffer[2] << 8)  |  (uint32_t)buffer[3];
      putreg32(regval, STM32_CRC_DR);
      buffer += sizeof(uint32_t);
      len    -= sizeof(uint32_t);
    }

  if (len == 1)
    {
      putreg8(buffer[0], STM32_CRC_DR);
    }
  else if (len == 2)
    {
      putreg16(((uint16_t)buffer[0] << 8) | buffer[1], STM32_CRC_DR);
    }
  else if (len == 3)
    {
      putreg16(((uint16_t)buffer[0] << 8) | buffer[1], STM32_CRC_DR);
      putreg8(buffer[2], STM32_CRC_DR);
    }

  *result = getreg32(STM32_CRC_DR);
  return OK;
}

#ifdef CONFIG_STM32_CRC_DMA
static void stm32_crc_dmacallback(DMA_HANDLE handle, uint8_t status,
                                  FAR void *arg)
{
  FAR struct stm32_crc_dev_s *priv = (FAR struct stm32_crc_dev_s *)arg;

  /* OR'ed with 0x80 to assure a non-zero value, distinguishing "posted"
   * from the pre-wait zero-initialized value.
   */

  priv->dmaresult = status | 0x80;
  nxsem_post(&priv->dmadone);
}

/****************************************************************************
 * Name: stm32_crc_write_dma
 *
 * Description:
 *   Feed the engine via GPDMA memory-to-peripheral (M2P) bursts using a
 *   software-triggered request (CRC has no DMAMUX hardware request
 *   line).  RESET is not pulsed between bursts within this call, so the
 *   engine's running value simply keeps accumulating across them, the
 *   same as consecutive stm32_crc_write_cpu() calls would.
 *
 *   NOTE: this M2P+SWREQ combination has no precedent elsewhere in the
 *   tree -- every other GPDMA consumer uses a hardware-paced REQSEL.
 *   It is architecturally supported (see stm32_dma.c's gpdma_setup())
 *   but should be treated as the least-proven part of this driver until
 *   validated on real hardware.
 *
 ****************************************************************************/

static int stm32_crc_write_dma(FAR struct crc_lowerhalf_s *lower,
                                FAR const uint8_t *buffer, size_t len,
                                FAR uint32_t *result)
{
  FAR struct stm32_crc_dev_s *priv =
    (FAR struct stm32_crc_dev_s *)lower->cl_priv;
  struct stm32_gpdma_cfg_s cfg;
  int ret;

  if (priv->dma == NULL)
    {
      return stm32_crc_write_cpu(lower, buffer, len, result);
    }

  memset(&cfg, 0, sizeof(cfg));
  cfg.dest_addr = STM32_CRC_DR;
  cfg.tr1       = GPDMA_CXTR1_SDW_LOG2_BYTE | GPDMA_CXTR1_SINC |
                   GPDMA_CXTR1_DDW_LOG2_BYTE;
  cfg.request   = GPDMA_CXTR2_SWREQ;
  cfg.priority  = GPDMACFG_PRIO_H;

  while (len > 0)
    {
      size_t chunk = len > UINT16_MAX ? UINT16_MAX : len;

      up_clean_dcache((uintptr_t)buffer, (uintptr_t)buffer + chunk);

      cfg.src_addr   = (uint32_t)(uintptr_t)buffer;
      cfg.ntransfers = (uint16_t)chunk;

      stm32_dmasetup(priv->dma, &cfg);

      priv->dmaresult = 0;
      stm32_dmastart(priv->dma, stm32_crc_dmacallback, priv, false);

      do
        {
          ret = nxsem_wait_uninterruptible(&priv->dmadone);
          DEBUGASSERT(ret == OK || ret == -ECANCELED);
        }
      while (priv->dmaresult == 0 && ret == OK);

      if (ret < 0)
        {
          return ret;
        }

      if ((priv->dmaresult & DMA_STATUS_FATAL) != 0)
        {
          _err("ERROR: CRC DMA transfer failed: status=%02x\n",
               priv->dmaresult);
          return -EIO;
        }

      buffer += chunk;
      len    -= chunk;
    }

  *result = getreg32(STM32_CRC_DR);
  return OK;
}
#endif /* CONFIG_STM32_CRC_DMA */

/****************************************************************************
 * Name: stm32_crc_write
 *
 * Description:
 *   The engine's single write entry point, wired into g_stm32_crc_ops.
 *   Chooses DMA or CPU writes internally, entirely transparently to the
 *   upper half -- the choice never changes the result, only how it is
 *   produced.
 *
 ****************************************************************************/

static int stm32_crc_write(FAR struct crc_lowerhalf_s *lower,
                           FAR const uint8_t *buffer, size_t len,
                           FAR uint32_t *result)
{
#ifdef CONFIG_STM32_CRC_DMA
  if (len >= CONFIG_STM32_CRC_DMA_THRESHOLD)
    {
      return stm32_crc_write_dma(lower, buffer, len, result);
    }
#endif

  return stm32_crc_write_cpu(lower, buffer, len, result);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct crc_lowerhalf_s *stm32h5_crc_initialize(void)
{
  g_stm32_crc_dev.lower.cl_ops  = &g_stm32_crc_ops;
  g_stm32_crc_dev.lower.cl_priv = &g_stm32_crc_dev;

#ifdef CONFIG_STM32_CRC_DMA
  nxsem_init(&g_stm32_crc_dev.dmadone, 0, 0);
#endif

  return &g_stm32_crc_dev.lower;
}
