/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_dcache.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DCACHE_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DCACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DCACHE1 characteristics (RM0481 Table 99) */

#define STM32_DCACHE_LINESIZE         16
#define STM32_DCACHE_SIZE             4096

/* Register offsets *********************************************************/

#define STM32_DCACHE_CR_OFFSET      0x0000  /* Control */
#define STM32_DCACHE_SR_OFFSET      0x0004  /* Status */
#define STM32_DCACHE_IER_OFFSET     0x0008  /* Interrupt enable */
#define STM32_DCACHE_FCR_OFFSET     0x000c  /* Flag clear */
#define STM32_DCACHE_RHMONR_OFFSET  0x0010  /* Read-hit monitor */
#define STM32_DCACHE_RMMONR_OFFSET  0x0014  /* Read-miss monitor */
#define STM32_DCACHE_WHMONR_OFFSET  0x0020  /* Write-hit monitor */
#define STM32_DCACHE_WMMONR_OFFSET  0x0024  /* Write-miss monitor */

/* Command range start and end addresses */

#define STM32_DCACHE_CMDRSADDRR_OFFSET 0x0028
#define STM32_DCACHE_CMDREADDRR_OFFSET 0x002c

/* Register addresses *******************************************************/

#define STM32_DCACHE_CR       (STM32_DCACHE_BASE + STM32_DCACHE_CR_OFFSET)
#define STM32_DCACHE_SR       (STM32_DCACHE_BASE + STM32_DCACHE_SR_OFFSET)
#define STM32_DCACHE_IER      (STM32_DCACHE_BASE + STM32_DCACHE_IER_OFFSET)
#define STM32_DCACHE_FCR      (STM32_DCACHE_BASE + STM32_DCACHE_FCR_OFFSET)
#define STM32_DCACHE_RHMONR   (STM32_DCACHE_BASE + STM32_DCACHE_RHMONR_OFFSET)
#define STM32_DCACHE_RMMONR   (STM32_DCACHE_BASE + STM32_DCACHE_RMMONR_OFFSET)
#define STM32_DCACHE_WHMONR   (STM32_DCACHE_BASE + STM32_DCACHE_WHMONR_OFFSET)
#define STM32_DCACHE_WMMONR   (STM32_DCACHE_BASE + STM32_DCACHE_WMMONR_OFFSET)
#define STM32_DCACHE_CMDRSADDRR \
          (STM32_DCACHE_BASE + STM32_DCACHE_CMDRSADDRR_OFFSET)
#define STM32_DCACHE_CMDREADDRR \
          (STM32_DCACHE_BASE + STM32_DCACHE_CMDREADDRR_OFFSET)

/* Register bit definitions *************************************************/

/* Control register */

#define DCACHE_CR_EN               (1 << 0)   /* Enable */
#define DCACHE_CR_CACHEINV         (1 << 1)   /* Full invalidation */
#define DCACHE_CR_CACHECMD_SHIFT   (8)        /* Bits 8-10: Range command */
#define DCACHE_CR_CACHECMD_MASK    (7 << DCACHE_CR_CACHECMD_SHIFT)
#  define DCACHE_CR_CACHECMD_CLEAN (1 << DCACHE_CR_CACHECMD_SHIFT)
#  define DCACHE_CR_CACHECMD_INV   (2 << DCACHE_CR_CACHECMD_SHIFT)
#  define DCACHE_CR_CACHECMD_FLUSH (3 << DCACHE_CR_CACHECMD_SHIFT)
#define DCACHE_CR_STARTCMD         (1 << 11)  /* Start range command */
#define DCACHE_CR_RHITMEN          (1 << 16)  /* Read-hit monitor enable */
#define DCACHE_CR_RMISSMEN         (1 << 17)  /* Read-miss monitor enable */
#define DCACHE_CR_RHITMRST         (1 << 18)  /* Read-hit monitor reset */
#define DCACHE_CR_RMISSMRST        (1 << 19)  /* Read-miss monitor reset */
#define DCACHE_CR_WHITMEN          (1 << 20)  /* Write-hit monitor enable */
#define DCACHE_CR_WMISSMEN         (1 << 21)  /* Write-miss monitor enable */
#define DCACHE_CR_WHITMRST         (1 << 22)  /* Write-hit monitor reset */
#define DCACHE_CR_WMISSMRST        (1 << 23)  /* Write-miss monitor reset */
#define DCACHE_CR_HBURST           (1u << 31) /* INCR (1) or WRAP refill */

/* Status register */

#define DCACHE_SR_BUSYF            (1 << 0)   /* Full invalidate busy */
#define DCACHE_SR_BSYENDF          (1 << 1)   /* Full invalidate done */
#define DCACHE_SR_ERRF             (1 << 2)   /* Cache error */
#define DCACHE_SR_BUSYCMDF         (1 << 3)   /* Range command busy */
#define DCACHE_SR_CMDENDF          (1 << 4)   /* Range command done */

/* Interrupt enable register */

#define DCACHE_IER_BSYENDIE        (1 << 1)   /* Full invalidate done */
#define DCACHE_IER_ERRIE           (1 << 2)   /* Cache error */
#define DCACHE_IER_CMDENDIE        (1 << 4)   /* Range command done */

/* Flag clear register */

#define DCACHE_FCR_CBSYENDF        (1 << 1)   /* Clear BSYENDF */
#define DCACHE_FCR_CERRF           (1 << 2)   /* Clear ERRF */
#define DCACHE_FCR_CCMDENDF        (1 << 4)   /* Clear CMDENDF */

/* Range address registers: bits 4-31 hold the line address */

#define DCACHE_CMDADDR_MASK        0xfffffff0

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DCACHE_H */
