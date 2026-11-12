/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dcache.c
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
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include <errno.h>
#include <stdint.h>

#include "arm_internal.h"
#include "stm32_dcache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A range command scans the 256 tags and may write back the whole 4 KiB
 * cache.  10 ms is a wide margin even for a slow external memory.
 */

#define DCACHE_TIMEOUT_US  10000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_dcache_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dcache_waitclear
 *
 * Description:
 *   Wait until the given DCACHE_SR bits are clear.
 *
 ****************************************************************************/

static int dcache_waitclear(uint32_t bits)
{
  unsigned int timeout;

  for (timeout = DCACHE_TIMEOUT_US;
       (getreg32(STM32_DCACHE_SR) & bits) != 0; timeout--)
    {
      if (timeout == 0)
        {
          return -ETIMEDOUT;
        }

      up_udelay(1);
    }

  return OK;
}

/****************************************************************************
 * Name: dcache_rangecmd
 *
 * Description:
 *   Run a clean and/or invalidate command over [start, end) and wait for
 *   it to finish.
 *
 ****************************************************************************/

static int dcache_rangecmd(uint32_t cmd, uintptr_t start, uintptr_t end)
{
  unsigned int timeout;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  if (end <= start)
    {
      return OK;
    }

  flags = spin_lock_irqsave(&g_dcache_lock);

  /* STARTCMD can only be set while the cache is enabled */

  regval = getreg32(STM32_DCACHE_CR);
  if ((regval & DCACHE_CR_EN) == 0)
    {
      ret = OK;
      goto out;
    }

  ret = dcache_waitclear(DCACHE_SR_BUSYF | DCACHE_SR_BUSYCMDF);
  if (ret < 0)
    {
      goto out;
    }

  /* Both addresses are line addresses and the end one is inclusive */

  putreg32(start & DCACHE_CMDADDR_MASK, STM32_DCACHE_CMDRSADDRR);
  putreg32((end - 1) & DCACHE_CMDADDR_MASK, STM32_DCACHE_CMDREADDRR);
  putreg32(DCACHE_FCR_CCMDENDF, STM32_DCACHE_FCR);

  regval &= ~DCACHE_CR_CACHECMD_MASK;
  putreg32(regval | cmd, STM32_DCACHE_CR);
  putreg32(regval | cmd | DCACHE_CR_STARTCMD, STM32_DCACHE_CR);

  /* BUSYCMDF may not be set yet right after STARTCMD; CMDENDF marks the
   * end of this command.
   */

  ret = -ETIMEDOUT;
  for (timeout = DCACHE_TIMEOUT_US; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_DCACHE_SR) & DCACHE_SR_CMDENDF) != 0)
        {
          putreg32(DCACHE_FCR_CCMDENDF, STM32_DCACHE_FCR);
          ret = OK;
          break;
        }

      up_udelay(1);
    }

out:
  spin_unlock_irqrestore(&g_dcache_lock, flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dcache_enable
 ****************************************************************************/

int stm32_dcache_enable(void)
{
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&g_dcache_lock);

  /* The cache invalidates itself after reset; wait for that to end */

  ret = dcache_waitclear(DCACHE_SR_BUSYF);
  if (ret == OK && (getreg32(STM32_DCACHE_CR) & DCACHE_CR_EN) == 0)
    {
      /* Refill with INCR bursts: a WRAP refill followed by a linear read
       * of the same OCTOSPI address returns wrong data (ES0565 2.5.10).
       * HBURST can only be changed while the cache is disabled.
       */

      modifyreg32(STM32_DCACHE_CR, 0, DCACHE_CR_HBURST);
      modifyreg32(STM32_DCACHE_CR, 0, DCACHE_CR_EN);
    }

  spin_unlock_irqrestore(&g_dcache_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: stm32_dcache_invalidate_all
 ****************************************************************************/

int stm32_dcache_invalidate_all(void)
{
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave(&g_dcache_lock);

  /* CACHEINV can only be set while the cache is enabled */

  if ((getreg32(STM32_DCACHE_CR) & DCACHE_CR_EN) != 0)
    {
      ret = dcache_waitclear(DCACHE_SR_BUSYF | DCACHE_SR_BUSYCMDF);
      if (ret == OK)
        {
          modifyreg32(STM32_DCACHE_CR, 0, DCACHE_CR_CACHEINV);

          /* BUSYF is set as soon as CACHEINV is written */

          ret = dcache_waitclear(DCACHE_SR_BUSYF);
          putreg32(DCACHE_FCR_CBSYENDF, STM32_DCACHE_FCR);
        }
    }

  spin_unlock_irqrestore(&g_dcache_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: stm32_dcache_invalidate
 ****************************************************************************/

int stm32_dcache_invalidate(uintptr_t start, uintptr_t end)
{
  return dcache_rangecmd(DCACHE_CR_CACHECMD_INV, start, end);
}

/****************************************************************************
 * Name: stm32_dcache_clean
 ****************************************************************************/

int stm32_dcache_clean(uintptr_t start, uintptr_t end)
{
  return dcache_rangecmd(DCACHE_CR_CACHECMD_CLEAN, start, end);
}

/****************************************************************************
 * Name: stm32_dcache_flush
 ****************************************************************************/

int stm32_dcache_flush(uintptr_t start, uintptr_t end)
{
  return dcache_rangecmd(DCACHE_CR_CACHECMD_FLUSH, start, end);
}
