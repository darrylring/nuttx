/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dlyb.c
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

#include <errno.h>
#include <stdint.h>

#include "arm_internal.h"
#include "stm32_dlyb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LNGF is set a few input clock cycles after UNIT changes; at the 25 MHz
 * minimum input clock this is well below 1 ms.
 */

#define DLYB_LNGF_TIMEOUT_US  1000

/* LNG bits used by the RM0481 27.4.4 length check */

#define DLYB_LNG_10_0         0x7ff
#define DLYB_LNG_11_10        0xc00

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dlyb_enable
 ****************************************************************************/

void stm32_dlyb_enable(uintptr_t base)
{
  putreg32(DLYB_CR_DEN | DLYB_CR_SEN, base + STM32_DLYB_CR_OFFSET);
}

/****************************************************************************
 * Name: stm32_dlyb_disable
 ****************************************************************************/

void stm32_dlyb_disable(uintptr_t base)
{
  putreg32(0, base + STM32_DLYB_CR_OFFSET);
}

/****************************************************************************
 * Name: stm32_dlyb_calibrate
 ****************************************************************************/

int stm32_dlyb_calibrate(uintptr_t base, uint8_t *unit, uint8_t *ncells)
{
  uintptr_t cfgr = base + STM32_DLYB_CFGR_OFFSET;
  unsigned int timeout;
  unsigned int u;
  unsigned int n;
  uint32_t lng;

  /* Enable the delay line and the length sampler, select all cells */

  stm32_dlyb_enable(base);

  for (u = 0; u <= DLYB_CFGR_UNIT_MAX; u++)
    {
      putreg32(DLYB_CFGR_SEL(DLYB_CFGR_SEL_MAX) | DLYB_CFGR_UNIT(u), cfgr);

      for (timeout = DLYB_LNGF_TIMEOUT_US;
           (getreg32(cfgr) & DLYB_CFGR_LNGF) == 0; timeout--)
        {
          if (timeout == 0)
            {
              return -ETIMEDOUT;
            }

          up_udelay(1);
        }

      lng = (getreg32(cfgr) & DLYB_CFGR_LNG_MASK) >> DLYB_CFGR_LNG_SHIFT;

      /* One period is covered when LNG[10:0] is not zero and LNG[11] and
       * LNG[10] are not both set.
       */

      if ((lng & DLYB_LNG_10_0) != 0 &&
          (lng & DLYB_LNG_11_10) != DLYB_LNG_11_10)
        {
          break;
        }
    }

  if (u > DLYB_CFGR_UNIT_MAX)
    {
      return -ERANGE;
    }

  /* The number of unit delays in one period is the highest set bit of
   * LNG[10:0].
   */

  for (n = 10; n > 0 && (lng & (1 << n)) == 0; n--)
    {
    }

  if (n == 0)
    {
      return -ERANGE;
    }

  /* Disable the length sampler */

  putreg32(DLYB_CR_DEN, base + STM32_DLYB_CR_OFFSET);

  *unit   = u;
  *ncells = n;
  return OK;
}

/****************************************************************************
 * Name: stm32_dlyb_setphase
 ****************************************************************************/

int stm32_dlyb_setphase(uintptr_t base, unsigned int sel)
{
  uintptr_t cfgr = base + STM32_DLYB_CFGR_OFFSET;
  uint32_t regval;

  if (sel > DLYB_CFGR_SEL_MAX)
    {
      return -EINVAL;
    }

  /* SEL can only be written while the sampler is enabled (SEN = 1) */

  putreg32(DLYB_CR_DEN | DLYB_CR_SEN, base + STM32_DLYB_CR_OFFSET);

  regval = getreg32(cfgr) & DLYB_CFGR_UNIT_MASK;
  putreg32(regval | DLYB_CFGR_SEL(sel), cfgr);

  /* Clearing SEN enables the output clock on the selected phase */

  putreg32(DLYB_CR_DEN, base + STM32_DLYB_CR_OFFSET);
  return OK;
}
