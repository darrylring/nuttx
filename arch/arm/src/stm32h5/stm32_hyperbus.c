/****************************************************************************
 * arch/arm/src/stm32h5/stm32_hyperbus.c
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
#include <nuttx/irq.h>
#include <nuttx/debug.h>

#include <sys/param.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <arch/barriers.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "mpu.h"
#include "stm32_dlyb.h"
#include "stm32_gpio.h"
#include "stm32_hyperbus.h"
#include "hardware/stm32_qspi.h"
#include "hardware/stm32_rcc.h"
#include "hardware/stm32_sbs.h"

#ifdef CONFIG_STM32_DCACHE1
#  include "stm32_dcache.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32_QSPI1
#  error "OCTOSPI1 cannot be used by both STM32_QSPI1 and STM32_HYPERBUS"
#endif

#if (STM32_HYPERBUS_SIZE & (STM32_HYPERBUS_SIZE - 1)) != 0
#  error "CONFIG_STM32_HYPERBUS_SIZE must be a power of two"
#endif

#if !defined(GPIO_OCTOSPI1_NCS) || !defined(GPIO_OCTOSPI1_CLK) || \
    !defined(GPIO_OCTOSPI1_DQS) || \
    !defined(GPIO_OCTOSPI1_IO0) || !defined(GPIO_OCTOSPI1_IO1) || \
    !defined(GPIO_OCTOSPI1_IO2) || !defined(GPIO_OCTOSPI1_IO3) || \
    !defined(GPIO_OCTOSPI1_IO4) || !defined(GPIO_OCTOSPI1_IO5) || \
    !defined(GPIO_OCTOSPI1_IO6) || !defined(GPIO_OCTOSPI1_IO7)
#  error "board.h must define GPIO_OCTOSPI1_NCS, _CLK, _DQS and _IO0-_IO7"
#endif

/* Polling limits.  The controller and the delay block respond within a few
 * CLK cycles; the I/O compensation cell and CSI take some microseconds.
 */

#define HYPERBUS_TIMEOUT_US   1000
#define IOCOMP_TIMEOUT_US     10000

/* A HyperBus transaction sends 48 command/address bits on eight DDR lines:
 * three CLK cycles.
 */

#define HYPERBUS_CA_CYCLES    3

/* REFRESH + 4 cycles is the longest read (RM0481 OCTOSPI_DCR4) */

#define HYPERBUS_READ_EXTRA   4

/* With a tCSM limit, CS# is also released after this many idle CLK cycles
 * (LPTR), so that a prefetch stalled with CS# low cannot exceed tCSM.
 */

#define HYPERBUS_IDLE_CYCLES  16

#define NSEC_PER_SEC_U64      1000000000ull

/* Memory-mapped read and write frame: command/address and data on eight
 * lines in DTR mode, DQS used for latency and strobing.
 */

#define HYPERBUS_CCR          (QSPI_CCR_DQSE | QSPI_CCR_DDTR | \
                               QSPI_CCR_DMODE(CCR_DMODE_OCTAL) | \
                               QSPI_CCR_ADDTR | QSPI_CCR_ADSIZE_32b | \
                               QSPI_CCR_ADMODE(CCR_ADMODE_OCTAL))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_hyperbus_pins[] =
{
  GPIO_OCTOSPI1_NCS, GPIO_OCTOSPI1_CLK, GPIO_OCTOSPI1_DQS,
  GPIO_OCTOSPI1_IO0, GPIO_OCTOSPI1_IO1, GPIO_OCTOSPI1_IO2,
  GPIO_OCTOSPI1_IO3, GPIO_OCTOSPI1_IO4, GPIO_OCTOSPI1_IO5,
  GPIO_OCTOSPI1_IO6, GPIO_OCTOSPI1_IO7,
#ifdef GPIO_OCTOSPI1_NCLK
  GPIO_OCTOSPI1_NCLK,  /* Only for 1.8 V memories */
#endif
};

static bool g_hyperbus_ready;
#ifdef CONFIG_STM32_HYPERBUS_MPU
static bool g_hyperbus_mpu;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hyperbus_waitreg
 *
 * Description:
 *   Wait until (*addr & mask) == value.
 *
 ****************************************************************************/

static int hyperbus_waitreg(uintptr_t addr, uint32_t mask, uint32_t value,
                            unsigned int timeout_us)
{
  while ((getreg32(addr) & mask) != value)
    {
      if (timeout_us-- == 0)
        {
          return -ETIMEDOUT;
        }

      up_udelay(1);
    }

  return OK;
}

/****************************************************************************
 * Name: hyperbus_waitidle
 ****************************************************************************/

static int hyperbus_waitidle(void)
{
  return hyperbus_waitreg(STM32_QUADSPI_SR, QSPI_SR_BUSY, 0,
                          HYPERBUS_TIMEOUT_US);
}

/****************************************************************************
 * Name: hyperbus_abort
 *
 * Description:
 *   Abort the current operation (memory-mapped prefetch or free-running
 *   clock) and wait until the controller is idle.
 *
 ****************************************************************************/

static int hyperbus_abort(void)
{
  int ret;

  modifyreg32(STM32_QUADSPI_CR, 0, QSPI_CR_ABORT);

  ret = hyperbus_waitreg(STM32_QUADSPI_CR, QSPI_CR_ABORT, 0,
                         HYPERBUS_TIMEOUT_US);
  if (ret == OK)
    {
      ret = hyperbus_waitidle();
    }

  return ret;
}

/****************************************************************************
 * Name: hyperbus_drain
 *
 * Description:
 *   Before an abort, let memory-mapped writes finish: write back DCACHE1,
 *   then do a read that reaches the controller and a barrier (RM0481
 *   24.4.19).  The OCTOSPI serves one AHB access at a time, so the read
 *   completes after the writes.
 *
 ****************************************************************************/

static void hyperbus_drain(void)
{
#ifdef CONFIG_STM32_DCACHE1
  stm32_dcache_flush(STM32_HYPERBUS_BASE,
                     STM32_HYPERBUS_BASE + STM32_HYPERBUS_SIZE);
#endif

  UP_DSB();
  (void)*(volatile uint32_t *)STM32_HYPERBUS_BASE;
  UP_DSB();
}

/****************************************************************************
 * Name: hyperbus_ns2clk
 *
 * Description:
 *   Convert a time in ns to CLK cycles, rounded up or down.
 *
 ****************************************************************************/

static uint32_t hyperbus_ns2clk(const struct stm32_hyperbus_config_s *cfg,
                                uint32_t ns, bool roundup)
{
  uint64_t num = (uint64_t)ns * cfg->kerfreq;
  uint64_t den = (uint64_t)cfg->prescaler * NSEC_PER_SEC_U64;

  if (roundup)
    {
      num += den - 1;
    }

  return (uint32_t)(num / den);
}

/****************************************************************************
 * Name: hyperbus_timings
 *
 * Description:
 *   Check the configuration and compute DCR1 CSHT and DCR4 REFRESH.
 *
 ****************************************************************************/

static int hyperbus_timings(const struct stm32_hyperbus_config_s *cfg,
                            uint32_t *csht, uint32_t *refresh)
{
  uint32_t cycles;
  uint32_t mincycles;

  if (cfg->kerfreq == 0 || cfg->prescaler < 1 || cfg->prescaler > 256 ||
      ((cfg->prescaler & 1) != 0 && cfg->prescaler > 1) ||
      cfg->tacc == 0 || cfg->csbound > 31 ||
      (cfg->dlybsel > DLYB_CFGR_SEL_MAX &&
       cfg->dlybsel != STM32_HYPERBUS_DLYBSEL_AUTO))
    {
      return -EINVAL;
    }

  /* CS# stays high for CSHT + 1 cycles: round tCSHI up */

  cycles = hyperbus_ns2clk(cfg, cfg->tcshi, true);
  if (cycles == 0)
    {
      cycles = 1;
    }

  if (cycles > 64)
    {
      return -EINVAL;
    }

  *csht = cycles - 1;

  /* A read keeps CS# low for up to REFRESH + 4 cycles, and an idle
   * prefetch for up to HYPERBUS_IDLE_CYCLES more: round tCSM down.
   * REFRESH must exceed the shortest transaction: command/address, the
   * worst-case 2 x tacc latency and one data cycle.
   */

  *refresh = 0;
  if (cfg->tcsm != 0)
    {
      cycles    = hyperbus_ns2clk(cfg, cfg->tcsm, false);
      mincycles = HYPERBUS_CA_CYCLES + 2 * cfg->tacc + 1;

      if (cycles <= HYPERBUS_READ_EXTRA + HYPERBUS_IDLE_CYCLES + mincycles)
        {
          return -EINVAL;
        }

      *refresh = cycles - HYPERBUS_READ_EXTRA - HYPERBUS_IDLE_CYCLES;
    }

  return OK;
}

/****************************************************************************
 * Name: hyperbus_iocomp
 *
 * Description:
 *   Enable the VDDIO compensation cell and apply its code (RM0481 14.3).
 *   The cell needs the CSI oscillator.
 *
 ****************************************************************************/

static int hyperbus_iocomp(void)
{
  int ret;

  if ((getreg32(STM32_RCC_CR) & RCC_CR_CSIRDY) == 0)
    {
      modifyreg32(STM32_RCC_CR, 0, RCC_CR_CSION);
      ret = hyperbus_waitreg(STM32_RCC_CR, RCC_CR_CSIRDY, RCC_CR_CSIRDY,
                             IOCOMP_TIMEOUT_US);
      if (ret < 0)
        {
          return ret;
        }
    }

  modifyreg32(STM32_SBS_CCCSR, 0, SBS_CCCSR_EN1);
  ret = hyperbus_waitreg(STM32_SBS_CCCSR, SBS_CCCSR_RDY1, SBS_CCCSR_RDY1,
                         IOCOMP_TIMEOUT_US);
  if (ret < 0)
    {
      return ret;
    }

  /* CS1 resets to 1 (code from SBS_CCSWCR); clear it to use the cell */

  modifyreg32(STM32_SBS_CCCSR, SBS_CCCSR_CS1, 0);
  return OK;
}

/****************************************************************************
 * Name: hyperbus_calibrate
 *
 * Description:
 *   Calibrate the delay block on the free-running CLK and select the DQS
 *   phase (RM0481 27.4.4, 27.4.5).  OCTOSPI1 must be disabled with DCR2
 *   set.
 *
 ****************************************************************************/

static int hyperbus_calibrate(const struct stm32_hyperbus_config_s *cfg)
{
  unsigned int sel;
  uint8_t ncells;
  uint8_t unit;
  int ret;
  int ret2;

  putreg32(0, STM32_QUADSPI_DCR1);
  putreg32(QSPI_CR_EN, STM32_QUADSPI_CR);

  ret = hyperbus_waitidle();
  if (ret < 0)
    {
      putreg32(0, STM32_QUADSPI_CR);
      return ret;
    }

  /* FRCK starts the clock and sets BUSY; only ABORT stops it */

  modifyreg32(STM32_QUADSPI_DCR1, 0, QSPI_DCR1_FRCK);

  ret = stm32_dlyb_calibrate(STM32_DLYBOS1_BASE, &unit, &ncells);

  ret2 = hyperbus_abort();
  modifyreg32(STM32_QUADSPI_DCR1, QSPI_DCR1_FRCK, 0);
  putreg32(0, STM32_QUADSPI_CR);

  if (ret == OK)
    {
      ret = ret2;
    }

  if (ret < 0)
    {
      merr("ERROR: Delay block calibration failed: %d\n", ret);
      return ret;
    }

  /* For DDR the sampling point is about a quarter period (RM0481 27.4.5) */

  if (cfg->dlybsel == STM32_HYPERBUS_DLYBSEL_AUTO)
    {
      sel = (ncells + 2) / 4;
    }
  else
    {
      sel = cfg->dlybsel;
    }

  minfo("DLYB UNIT %u, %u cells per period, SEL %u\n", unit, ncells, sel);

  return stm32_dlyb_setphase(STM32_DLYBOS1_BASE, sel);
}

/****************************************************************************
 * Name: hyperbus_mpu
 *
 * Description:
 *   Map the memory as Normal memory, read/write for all, outer shareable.
 *   It is write-back cacheable when DCACHE1 is used.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_HYPERBUS_MPU
static void hyperbus_mpu(void)
{
  if (g_hyperbus_mpu)
    {
      return;
    }

  mpu_configure_region(STM32_HYPERBUS_BASE, STM32_HYPERBUS_SIZE,
                       MPU_RBAR_AP_RWRW | MPU_RBAR_SH_OUTER,
#ifdef CONFIG_STM32_DCACHE1
                       MPU_RLAR_WRITE_BACK
#else
                       MPU_RLAR_NONCACHEABLE
#endif
                       );

  g_hyperbus_mpu = true;
}
#else
#  define hyperbus_mpu()
#endif

/****************************************************************************
 * Name: hyperbus_regaccess
 *
 * Description:
 *   Read or write one register in HyperBus register mode with an indirect
 *   transfer, then return to memory-mapped mode.
 *
 ****************************************************************************/

static int hyperbus_regaccess(uint32_t addr, uint16_t *val, bool write)
{
  unsigned int timeout;
  irqstate_t flags;
  uint32_t dcr1;
  uint32_t hlcr;
  uint32_t cr;
  uint32_t sr;
  int ret;
  int ret2;

  if (!g_hyperbus_ready)
    {
      return -ENODEV;
    }

  if ((addr & 1) != 0)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  hyperbus_drain();

  ret = hyperbus_abort();
  if (ret < 0)
    {
      goto out;
    }

  dcr1 = getreg32(STM32_QUADSPI_DCR1);
  hlcr = getreg32(STM32_QUADSPI_HLCR);
  cr   = getreg32(STM32_QUADSPI_CR);

  putreg32((dcr1 & ~QSPI_DCR1_MTYP_MASK) | QSPI_DCR1_MTYP_HYPBUS_REG,
           STM32_QUADSPI_DCR1);

  /* Register writes carry no latency */

  if (write)
    {
      putreg32(hlcr | QSPI_HLCR_WZL, STM32_QUADSPI_HLCR);
    }

  putreg32((cr & ~QSPI_CR_FMODE_MASK) |
           QSPI_CR_FMODE(write ? CR_FMODE_INDWR : CR_FMODE_INDRD),
           STM32_QUADSPI_CR);

  putreg32(QSPI_FCR_CTCF | QSPI_FCR_CTEF, STM32_QUADSPI_FCR);
  putreg32(sizeof(uint16_t) - 1, STM32_QUADSPI_DLR);

  /* A read starts on the AR write, a write on the DR write */

  putreg32(addr, STM32_QUADSPI_AR);
  if (write)
    {
      putreg16(*val, STM32_QUADSPI_DR);
    }

  ret = -ETIMEDOUT;
  for (timeout = HYPERBUS_TIMEOUT_US; timeout > 0; timeout--)
    {
      sr = getreg32(STM32_QUADSPI_SR);
      if ((sr & (QSPI_SR_TCF | QSPI_SR_TEF)) != 0)
        {
          ret = (sr & QSPI_SR_TEF) != 0 ? -EIO : OK;
          break;
        }

      up_udelay(1);
    }

  if (ret == OK && !write)
    {
      *val = getreg16(STM32_QUADSPI_DR);
    }

  putreg32(QSPI_FCR_CTCF | QSPI_FCR_CTEF, STM32_QUADSPI_FCR);

  /* Back to memory-mapped mode, even after an error */

  ret2 = ret == OK ? hyperbus_waitidle() : hyperbus_abort();
  putreg32(dcr1, STM32_QUADSPI_DCR1);
  putreg32(hlcr, STM32_QUADSPI_HLCR);
  putreg32(cr, STM32_QUADSPI_CR);

  if (ret == OK)
    {
      ret = ret2;
    }

  ret2 = hyperbus_waitidle();
  if (ret == OK)
    {
      ret = ret2;
    }

  UP_DSB();
  UP_ISB();

out:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hyperbus_initialize
 ****************************************************************************/

int stm32_hyperbus_initialize(const struct stm32_hyperbus_config_s *cfg)
{
  unsigned int devsize;
  unsigned int i;
  uint32_t refresh;
  uint32_t csht;
  uint32_t regval;
  int ret;

  DEBUGASSERT(cfg != NULL);

  ret = hyperbus_timings(cfg, &csht, &refresh);
  if (ret < 0)
    {
      merr("ERROR: Invalid HyperBus configuration\n");
      return ret;
    }

  /* Stop a previous configuration; write back cached data first */

  if (g_hyperbus_ready)
    {
      g_hyperbus_ready = false;
      hyperbus_drain();
      hyperbus_abort();
    }

  for (i = 0; i < nitems(g_hyperbus_pins); i++)
    {
      stm32_configgpio(g_hyperbus_pins[i]);
    }

  if (cfg->iocomp)
    {
      ret = hyperbus_iocomp();
      if (ret < 0)
        {
          merr("ERROR: I/O compensation cell not ready\n");
          return ret;
        }
    }

  /* Clock and reset OCTOSPI1.  The delay block registers need its clock. */

  modifyreg32(STM32_RCC_AHB4ENR, 0, RCC_AHB4ENR_OSPI1EN);
  modifyreg32(STM32_RCC_AHB4RSTR, 0, RCC_AHB4RSTR_OSPI1RST);
  modifyreg32(STM32_RCC_AHB4RSTR, RCC_AHB4RSTR_OSPI1RST, 0);

  putreg32(QSPI_DCR2_PRESCALER(cfg->prescaler - 1), STM32_QUADSPI_DCR2);

  if (cfg->usedlyb)
    {
      ret = hyperbus_calibrate(cfg);
      if (ret < 0)
        {
          stm32_dlyb_disable(STM32_DLYBOS1_BASE);
          return ret;
        }
    }
  else
    {
      stm32_dlyb_disable(STM32_DLYBOS1_BASE);
    }

  /* Device configuration.  DEVSIZE + 1 is the number of address bits. */

  for (devsize = 0; (2u << devsize) < STM32_HYPERBUS_SIZE; devsize++)
    {
    }

  regval = QSPI_DCR1_MTYP_HYPBUS_MEM | QSPI_DCR1_DEVSIZE(devsize) |
           QSPI_DCR1_CSHT(csht);
  if (!cfg->usedlyb)
    {
      regval |= QSPI_DCR1_DLYBYP;
    }

  putreg32(regval, STM32_QUADSPI_DCR1);
  putreg32(QSPI_DCR3_CSBOUND(cfg->csbound), STM32_QUADSPI_DCR3);
  putreg32(refresh, STM32_QUADSPI_DCR4);

  /* HyperBus latency; TACC and TRWR are plain cycle counts */

  regval = QSPI_HLCR_TRWR(cfg->trwr) | QSPI_HLCR_TACC(cfg->tacc);
  if (cfg->wzl)
    {
      regval |= QSPI_HLCR_WZL;
    }

  if (cfg->fixedlat)
    {
      regval |= QSPI_HLCR_LM;
    }

  putreg32(regval, STM32_QUADSPI_HLCR);

  /* The latency comes from HLCR, so no dummy cycles.  SSHIFT must stay
   * clear in DTR mode.
   */

  putreg32(cfg->dhqc ? QSPI_TCR_DHQC : 0, STM32_QUADSPI_TCR);
  putreg32(0, STM32_QUADSPI_WTCR);
  putreg32(HYPERBUS_CCR, STM32_QUADSPI_CCR);
  putreg32(HYPERBUS_CCR, STM32_QUADSPI_WCCR);

  /* With a tCSM limit, release CS# when the bus is idle (TCEN) */

  putreg32(refresh != 0 ? HYPERBUS_IDLE_CYCLES : 0, STM32_QUADSPI_LPTR);

  /* The MPU attributes must be in place before the first access */

  hyperbus_mpu();

  putreg32(QSPI_CR_FMODE(CR_FMODE_MEMMAP) | QSPI_CR_EN |
           (refresh != 0 ? QSPI_CR_TCEN : 0), STM32_QUADSPI_CR);

  /* No access before the configuration is complete (RM0481 24.4.15) */

  ret = hyperbus_waitidle();
  UP_DSB();
  UP_ISB();
  if (ret < 0)
    {
      merr("ERROR: OCTOSPI1 stays busy\n");
      return ret;
    }

#ifdef CONFIG_STM32_DCACHE1
  ret = stm32_dcache_enable();
  if (ret < 0)
    {
      merr("ERROR: DCACHE1 enable failed\n");
      return ret;
    }
#endif

  g_hyperbus_ready = true;
  return OK;
}

/****************************************************************************
 * Name: stm32_hyperbus_ready
 ****************************************************************************/

bool stm32_hyperbus_ready(void)
{
  return g_hyperbus_ready;
}

/****************************************************************************
 * Name: stm32_hyperbus_readreg
 ****************************************************************************/

int stm32_hyperbus_readreg(uint32_t addr, uint16_t *val)
{
  DEBUGASSERT(val != NULL);
  return hyperbus_regaccess(addr, val, false);
}

/****************************************************************************
 * Name: stm32_hyperbus_writereg
 ****************************************************************************/

int stm32_hyperbus_writereg(uint32_t addr, uint16_t val)
{
  return hyperbus_regaccess(addr, &val, true);
}
