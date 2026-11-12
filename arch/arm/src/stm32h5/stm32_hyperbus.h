/****************************************************************************
 * arch/arm/src/stm32h5/stm32_hyperbus.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_HYPERBUS_H
#define __ARCH_ARM_SRC_STM32H5_STM32_HYPERBUS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "chip.h"

#ifdef CONFIG_STM32_HYPERBUS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The HyperRAM is mapped at the OCTOSPI1 memory window */

#define STM32_HYPERBUS_BASE       STM32_OCTOSPI1_BANK
#define STM32_HYPERBUS_SIZE       CONFIG_STM32_HYPERBUS_SIZE

/* Use dlybsel = STM32_HYPERBUS_DLYBSEL_AUTO to delay DQS by a quarter of
 * the calibrated clock period, the RM0481 27.4.5 value for DDR.
 */

#define STM32_HYPERBUS_DLYBSEL_AUTO 0xff

/* HyperRAM register space addresses as OCTOSPI byte addresses.  The
 * controller sends AR / 2 as the HyperBus word address, so ID1 at word
 * 0x0001 is byte 0x0002 and CR0 at word 0x0800 is byte 0x1000.
 */

#define HYPERRAM_ID0              0x0000
#define HYPERRAM_ID1              0x0002
#define HYPERRAM_CR0              0x1000
#define HYPERRAM_CR1              0x1002

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timings come from the memory data sheet.  The driver converts the times
 * in ns to CLK cycles, rounding to the safe side.
 */

struct stm32_hyperbus_config_s
{
  uint32_t kerfreq;   /* OCTOSPI1 kernel clock in Hz (RCC_CCIPR4) */
  uint16_t prescaler; /* CLK = kerfreq / prescaler: 1 or even, max 256 */
  uint8_t  tacc;      /* Initial latency in CLK cycles (HLCR TACC) */
  uint8_t  trwr;      /* Read-write recovery in CLK cycles (HLCR TRWR) */
  uint16_t tcshi;     /* Minimum CS# high time in ns (DCR1 CSHT) */
  uint16_t tcsm;      /* Maximum CS# low time in ns (DCR4), 0 = no limit */
  uint8_t  csbound;   /* Release CS# every 2^csbound bytes, 0 = off */
  uint8_t  dlybsel;   /* DQS delay in unit delays, or DLYBSEL_AUTO */
  bool     fixedlat;  /* Always wait 2 x tacc (HLCR LM) */
  bool     wzl;       /* No latency on memory writes (HLCR WZL) */
  bool     dhqc;      /* Hold write data a quarter cycle (TCR DHQC) */
  bool     iocomp;    /* Enable the VDDIO I/O compensation cell */
  bool     usedlyb;   /* Calibrate the delay block and delay DQS */
};

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hyperbus_initialize
 *
 * Description:
 *   Configure OCTOSPI1 for a HyperBus memory and map it at
 *   STM32_HYPERBUS_BASE.  The board selects the OCTOSPI1 kernel clock in
 *   board.h and resets the memory through its RESET# pin before this call.
 *   The function busy-waits and may run from stm32_boardinitialize().
 *   With CONFIG_STM32_DCACHE1 it also enables DCACHE1.
 *
 *   Calling it again reconfigures the controller; nothing may access the
 *   memory meanwhile.
 *
 * Input Parameters:
 *   cfg - Memory timings and options
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure: -EINVAL for a
 *   timing that does not fit the registers, -ETIMEDOUT if the hardware
 *   does not respond, -ERANGE if the delay block cannot be calibrated.
 *
 ****************************************************************************/

int stm32_hyperbus_initialize(const struct stm32_hyperbus_config_s *cfg);

/****************************************************************************
 * Name: stm32_hyperbus_ready
 *
 * Description:
 *   Return true once stm32_hyperbus_initialize() has succeeded.
 *
 ****************************************************************************/

bool stm32_hyperbus_ready(void);

/****************************************************************************
 * Name: stm32_hyperbus_readreg
 *
 * Description:
 *   Read a 16-bit register of the memory register space, for example
 *   HYPERRAM_ID0.  Memory-mapped mode is stopped meanwhile, with interrupts
 *   disabled; no DMA may access the memory during the call.
 *
 * Input Parameters:
 *   addr - Register byte address (even)
 *   val  - Returns the register value
 *
 * Returned Value:
 *   Zero on success; -ENODEV before initialization, -EINVAL for an odd
 *   address, -EIO on a transfer error, -ETIMEDOUT on timeout.
 *
 ****************************************************************************/

int stm32_hyperbus_readreg(uint32_t addr, uint16_t *val);

/****************************************************************************
 * Name: stm32_hyperbus_writereg
 *
 * Description:
 *   Write a 16-bit register of the memory register space, for example
 *   HYPERRAM_CR0.  The write is sent without latency, as the HyperBus
 *   register writes require.  If the write changes the memory latency,
 *   call stm32_hyperbus_initialize() again with matching timings.
 *
 * Input Parameters:
 *   addr - Register byte address (even)
 *   val  - Value to write
 *
 * Returned Value:
 *   As for stm32_hyperbus_readreg().
 *
 ****************************************************************************/

int stm32_hyperbus_writereg(uint32_t addr, uint16_t val);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32_HYPERBUS */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_HYPERBUS_H */
