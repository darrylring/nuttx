/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dcache.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_DCACHE_H
#define __ARCH_ARM_SRC_STM32H5_STM32_DCACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/stm32_dcache.h"

/* DCACHE1 sits on the Cortex-M33 S-AHB bus and only caches the external
 * memory windows (OCTOSPI and FMC, RM0481 9.3).  It is not the CPU cache
 * that up_invalidate_dcache() and friends manage, so drivers that share
 * external memory with a DMA must call the functions below.  The range
 * functions act on whole 16-byte lines: keep DMA buffers line aligned.
 */

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dcache_enable
 *
 * Description:
 *   Enable DCACHE1, with INCR refill bursts.  Configure the MPU attributes
 *   of the cached memory first: only accesses marked cacheable are cached.
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT if the reset invalidation never ends.
 *
 ****************************************************************************/

int stm32_dcache_enable(void);

/****************************************************************************
 * Name: stm32_dcache_invalidate_all
 *
 * Description:
 *   Invalidate all of DCACHE1.  Dirty lines are discarded, not written
 *   back.  Does nothing if DCACHE1 is disabled.
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT on timeout.
 *
 ****************************************************************************/

int stm32_dcache_invalidate_all(void);

/****************************************************************************
 * Name: stm32_dcache_invalidate
 *
 * Description:
 *   Invalidate the lines of DCACHE1 that hold [start, end).  Dirty lines
 *   are discarded.  Use before the CPU reads data that a DMA wrote.
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT on timeout.
 *
 ****************************************************************************/

int stm32_dcache_invalidate(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Name: stm32_dcache_clean
 *
 * Description:
 *   Write back the dirty lines of DCACHE1 that hold [start, end).  Use
 *   before a DMA reads data that the CPU wrote.
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT on timeout.
 *
 ****************************************************************************/

int stm32_dcache_clean(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Name: stm32_dcache_flush
 *
 * Description:
 *   Write back, then invalidate the lines of DCACHE1 that hold
 *   [start, end).
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT on timeout.
 *
 ****************************************************************************/

int stm32_dcache_flush(uintptr_t start, uintptr_t end);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_DCACHE_H */
