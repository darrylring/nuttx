/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_dlyb.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DLYB_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DLYB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define STM32_DLYB_CR_OFFSET      0x0000  /* Control register */
#define STM32_DLYB_CFGR_OFFSET    0x0004  /* Configuration register */

/* Register bit definitions *************************************************/

/* Control register */

#define DLYB_CR_DEN               (1 << 0)  /* Bit 0: Delay block enable */
#define DLYB_CR_SEN               (1 << 1)  /* Bit 1: Sampler length enable */

/* Configuration register */

#define DLYB_CFGR_SEL_SHIFT       (0)       /* Bits 0-3: Output clock phase */
#define DLYB_CFGR_SEL_MASK        (0xf << DLYB_CFGR_SEL_SHIFT)
#  define DLYB_CFGR_SEL(n)        ((uint32_t)(n) << DLYB_CFGR_SEL_SHIFT)
#define DLYB_CFGR_SEL_MAX         12        /* All delay cells selected */
#define DLYB_CFGR_UNIT_SHIFT      (8)       /* Bits 8-14: Unit delay */
#define DLYB_CFGR_UNIT_MASK       (0x7f << DLYB_CFGR_UNIT_SHIFT)
#  define DLYB_CFGR_UNIT(n)       ((uint32_t)(n) << DLYB_CFGR_UNIT_SHIFT)
#define DLYB_CFGR_UNIT_MAX        127
#define DLYB_CFGR_LNG_SHIFT       (16)      /* Bits 16-27: Delay line length */
#define DLYB_CFGR_LNG_MASK        (0xfff << DLYB_CFGR_LNG_SHIFT)
#define DLYB_CFGR_LNGF            (1u << 31) /* Bit 31: Length valid flag */

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DLYB_H */
