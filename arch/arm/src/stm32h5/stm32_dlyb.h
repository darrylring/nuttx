/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dlyb.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_DLYB_H
#define __ARCH_ARM_SRC_STM32H5_STM32_DLYB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/stm32_dlyb.h"

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dlyb_enable
 *
 * Description:
 *   Enable a delay block.  The output clock stays disabled until a phase is
 *   selected with stm32_dlyb_setphase().
 *
 * Input Parameters:
 *   base - Delay block base address, for example STM32_DLYBOS1_BASE
 *
 ****************************************************************************/

void stm32_dlyb_enable(uintptr_t base);

/****************************************************************************
 * Name: stm32_dlyb_disable
 *
 * Description:
 *   Disable a delay block.  The output clock is then the input clock.
 *
 * Input Parameters:
 *   base - Delay block base address
 *
 ****************************************************************************/

void stm32_dlyb_disable(uintptr_t base);

/****************************************************************************
 * Name: stm32_dlyb_calibrate
 *
 * Description:
 *   Find the unit delay for which the delay line spans one input clock
 *   period (RM0481 27.4.4).  The input clock must run continuously while
 *   this function runs.  The delay block is enabled and the unit delay is
 *   kept in DLYB_CFGR.
 *
 * Input Parameters:
 *   base   - Delay block base address
 *   unit   - Returns the unit delay (UNIT)
 *   ncells - Returns the number of unit delays in one input clock period
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT if the length flag is never set (no input
 *   clock); -ERANGE if no unit delay spans one period.
 *
 ****************************************************************************/

int stm32_dlyb_calibrate(uintptr_t base, uint8_t *unit, uint8_t *ncells);

/****************************************************************************
 * Name: stm32_dlyb_setphase
 *
 * Description:
 *   Select the output clock phase in unit delays (RM0481 27.4.5) and enable
 *   the output clock.  The unit delay is not changed.
 *
 * Input Parameters:
 *   base - Delay block base address
 *   sel  - Output clock phase, 0 to DLYB_CFGR_SEL_MAX
 *
 * Returned Value:
 *   Zero on success; -EINVAL if sel is out of range.
 *
 ****************************************************************************/

int stm32_dlyb_setphase(uintptr_t base, unsigned int sel);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_DLYB_H */
