/****************************************************************************
 * include/nuttx/motor/gate_driver/drv83xx_core.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_GATE_DRIVER_DRV83XX_CORE_H
#define __INCLUDE_NUTTX_MOTOR_GATE_DRIVER_DRV83XX_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers (4-bit addresses, MSB of 11-bit data is bit 10) */

#define DRV83XX_REG_FAULT_STATUS1       (0x00)    /* Fault Status 1 (R) */
#define DRV83XX_REG_VGS_STATUS2         (0x01)    /* VGS Status 2 (R) */
#define DRV83XX_REG_DRIVER_CONTROL      (0x02)    /* Driver Control (R/W) */
#define DRV83XX_REG_GATE_DRIVE_HS       (0x03)    /* Gate Drive HS (R/W) */
#define DRV83XX_REG_GATE_DRIVE_LS       (0x04)    /* Gate Drive LS (R/W) */
#define DRV83XX_REG_OCP_CONTROL         (0x05)    /* OCP Control (R/W) */
#define DRV83XX_REG_CSA_CONTROL         (0x06)    /* CSA Control (R/W) */

/* SPI frame helpers (16-bit: W[15] | A[14:11] | D[10:0]) */

#define DRV83XX_SPI_W_READ              (1u << 15)
#define DRV83XX_SPI_W_WRITE             (0u << 15)
#define DRV83XX_SPI_ADDR_SHIFT          (11)
#define DRV83XX_SPI_ADDR_MASK           (0xfu << DRV83XX_SPI_ADDR_SHIFT)
#define DRV83XX_SPI_DATA_MASK           (0x7ffu)

/* Fault Status Register 1 (0x00, read-only) */

#define DRV83XX_FS1_VDS_LC              (1u << 0)  /* VDS OC fault on C low-side  */
#define DRV83XX_FS1_VDS_HC              (1u << 1)  /* VDS OC fault on C high-side */
#define DRV83XX_FS1_VDS_LB              (1u << 2)  /* VDS OC fault on B low-side  */
#define DRV83XX_FS1_VDS_HB              (1u << 3)  /* VDS OC fault on B high-side */
#define DRV83XX_FS1_VDS_LA              (1u << 4)  /* VDS OC fault on A low-side  */
#define DRV83XX_FS1_VDS_HA              (1u << 5)  /* VDS OC fault on A high-side */
#define DRV83XX_FS1_OTSD                (1u << 6)  /* Overtemperature shutdown */
#define DRV83XX_FS1_UVLO                (1u << 7)  /* VM undervoltage lockout */
#define DRV83XX_FS1_GDF                 (1u << 8)  /* Gate drive fault */
#define DRV83XX_FS1_VDS_OCP             (1u << 9)  /* VDS OC fault (any) */
#define DRV83XX_FS1_FAULT               (1u << 10) /* OR of all fault bits */

/* VGS Status Register 2 (0x01, read-only) */

#define DRV83XX_FS2_VGS_LC              (1u << 0)  /* Gate fault C low-side  */
#define DRV83XX_FS2_VGS_HC              (1u << 1)  /* Gate fault C high-side */
#define DRV83XX_FS2_VGS_LB              (1u << 2)  /* Gate fault B low-side  */
#define DRV83XX_FS2_VGS_HB              (1u << 3)  /* Gate fault B high-side */
#define DRV83XX_FS2_VGS_LA              (1u << 4)  /* Gate fault A low-side  */
#define DRV83XX_FS2_VGS_HA              (1u << 5)  /* Gate fault A high-side */
#define DRV83XX_FS2_CPUV                (1u << 6)  /* Charge-pump undervoltage */
#define DRV83XX_FS2_OTW                 (1u << 7)  /* Overtemperature warning */
#define DRV83XX_FS2_SC_OC               (1u << 8)  /* Sense OC on phase C */
#define DRV83XX_FS2_SB_OC               (1u << 9)  /* Sense OC on phase B */
#define DRV83XX_FS2_SA_OC               (1u << 10) /* Sense OC on phase A */

/* Driver Control Register (0x02, R/W) */

#define DRV83XX_DRVCTL_CLR_FLT          (1u << 0)  /* Clear latched fault bits */
#define DRV83XX_DRVCTL_BRAKE            (1u << 1)  /* Brake (1x PWM mode) */
#define DRV83XX_DRVCTL_COAST            (1u << 2)  /* Hi-Z all MOSFETs */
#define DRV83XX_DRVCTL_1PWM_DIR         (1u << 3)  /* 1x PWM direction */
#define DRV83XX_DRVCTL_1PWM_COM         (1u << 4)  /* 1x PWM async rectification */
#define DRV83XX_DRVCTL_PWMMODE_SHIFT    (5)
#define DRV83XX_DRVCTL_PWMMODE_MASK     (0x3u << DRV83XX_DRVCTL_PWMMODE_SHIFT)
#  define DRV83XX_DRVCTL_PWMMODE_6X     (0x0u << DRV83XX_DRVCTL_PWMMODE_SHIFT)
#  define DRV83XX_DRVCTL_PWMMODE_3X     (0x1u << DRV83XX_DRVCTL_PWMMODE_SHIFT)
#  define DRV83XX_DRVCTL_PWMMODE_1X     (0x2u << DRV83XX_DRVCTL_PWMMODE_SHIFT)
#  define DRV83XX_DRVCTL_PWMMODE_IND    (0x3u << DRV83XX_DRVCTL_PWMMODE_SHIFT)
#define DRV83XX_DRVCTL_PWMMODE(x)       (((x) << DRV83XX_DRVCTL_PWMMODE_SHIFT) & \
                                         DRV83XX_DRVCTL_PWMMODE_MASK)
#define DRV83XX_DRVCTL_OTW_REP          (1u << 7)  /* Report OTW on nFAULT */
#define DRV83XX_DRVCTL_DIS_GDF          (1u << 8)  /* Disable gate-drive fault */
#define DRV83XX_DRVCTL_DIS_CPUV         (1u << 9)  /* Disable charge-pump UVLO */
/* bit 10 reserved */

/* Gate Drive HS Register (0x03, R/W) */

#define DRV83XX_GDHS_IDRIVEN_HS_SHIFT   (0)
#define DRV83XX_GDHS_IDRIVEN_HS_MASK    (0xfu << DRV83XX_GDHS_IDRIVEN_HS_SHIFT)
#define DRV83XX_GDHS_IDRIVEN_HS(x)      (((x) << DRV83XX_GDHS_IDRIVEN_HS_SHIFT) & \
                                         DRV83XX_GDHS_IDRIVEN_HS_MASK)
#define DRV83XX_GDHS_IDRIVEP_HS_SHIFT   (4)
#define DRV83XX_GDHS_IDRIVEP_HS_MASK    (0xfu << DRV83XX_GDHS_IDRIVEP_HS_SHIFT)
#define DRV83XX_GDHS_IDRIVEP_HS(x)      (((x) << DRV83XX_GDHS_IDRIVEP_HS_SHIFT) & \
                                         DRV83XX_GDHS_IDRIVEP_HS_MASK)
#define DRV83XX_GDHS_LOCK_SHIFT         (8)
#define DRV83XX_GDHS_LOCK_MASK          (0x7u << DRV83XX_GDHS_LOCK_SHIFT)
#  define DRV83XX_GDHS_LOCK_UNLOCKED    (0x3u << DRV83XX_GDHS_LOCK_SHIFT)
#  define DRV83XX_GDHS_LOCK_LOCKED      (0x6u << DRV83XX_GDHS_LOCK_SHIFT)

/* Gate Drive LS Register (0x04, R/W) */

#define DRV83XX_GDLS_IDRIVEN_LS_SHIFT   (0)
#define DRV83XX_GDLS_IDRIVEN_LS_MASK    (0xfu << DRV83XX_GDLS_IDRIVEN_LS_SHIFT)
#define DRV83XX_GDLS_IDRIVEN_LS(x)      (((x) << DRV83XX_GDLS_IDRIVEN_LS_SHIFT) & \
                                         DRV83XX_GDLS_IDRIVEN_LS_MASK)
#define DRV83XX_GDLS_IDRIVEP_LS_SHIFT   (4)
#define DRV83XX_GDLS_IDRIVEP_LS_MASK    (0xfu << DRV83XX_GDLS_IDRIVEP_LS_SHIFT)
#define DRV83XX_GDLS_IDRIVEP_LS(x)      (((x) << DRV83XX_GDLS_IDRIVEP_LS_SHIFT) & \
                                         DRV83XX_GDLS_IDRIVEP_LS_MASK)
#define DRV83XX_GDLS_TDRIVE_SHIFT       (8)
#define DRV83XX_GDLS_TDRIVE_MASK        (0x3u << DRV83XX_GDLS_TDRIVE_SHIFT)
#  define DRV83XX_GDLS_TDRIVE_500NS     (0x0u << DRV83XX_GDLS_TDRIVE_SHIFT)
#  define DRV83XX_GDLS_TDRIVE_1US       (0x1u << DRV83XX_GDLS_TDRIVE_SHIFT)
#  define DRV83XX_GDLS_TDRIVE_2US       (0x2u << DRV83XX_GDLS_TDRIVE_SHIFT)
#  define DRV83XX_GDLS_TDRIVE_4US       (0x3u << DRV83XX_GDLS_TDRIVE_SHIFT)
#define DRV83XX_GDLS_TDRIVE(x)          (((x) << DRV83XX_GDLS_TDRIVE_SHIFT) & \
                                         DRV83XX_GDLS_TDRIVE_MASK)
#define DRV83XX_GDLS_CBC                (1u << 10) /* Cycle-by-cycle OC retry */

/* OCP Control Register (0x05, R/W) */

#define DRV83XX_OCP_VDSLVL_SHIFT        (0)
#define DRV83XX_OCP_VDSLVL_MASK         (0xfu << DRV83XX_OCP_VDSLVL_SHIFT)
#  define DRV83XX_OCP_VDSLVL_0V06       (0x0u)
#  define DRV83XX_OCP_VDSLVL_0V13       (0x1u)
#  define DRV83XX_OCP_VDSLVL_0V20       (0x2u)
#  define DRV83XX_OCP_VDSLVL_0V26       (0x3u)
#  define DRV83XX_OCP_VDSLVL_0V31       (0x4u)
#  define DRV83XX_OCP_VDSLVL_0V45       (0x5u)
#  define DRV83XX_OCP_VDSLVL_0V53       (0x6u)
#  define DRV83XX_OCP_VDSLVL_0V60       (0x7u)
#  define DRV83XX_OCP_VDSLVL_0V68       (0x8u)
#  define DRV83XX_OCP_VDSLVL_0V75       (0x9u)
#  define DRV83XX_OCP_VDSLVL_0V94       (0xau)
#  define DRV83XX_OCP_VDSLVL_1V13       (0xbu)
#  define DRV83XX_OCP_VDSLVL_1V30       (0xcu)
#  define DRV83XX_OCP_VDSLVL_1V50       (0xdu)
#  define DRV83XX_OCP_VDSLVL_1V70       (0xeu)
#  define DRV83XX_OCP_VDSLVL_1V88       (0xfu)
#define DRV83XX_OCP_VDSLVL(x)           (((x) << DRV83XX_OCP_VDSLVL_SHIFT) & \
                                         DRV83XX_OCP_VDSLVL_MASK)
#define DRV83XX_OCP_OCPDEG_SHIFT        (4)
#define DRV83XX_OCP_OCPDEG_MASK         (0x3u << DRV83XX_OCP_OCPDEG_SHIFT)
#  define DRV83XX_OCP_OCPDEG_2US        (0x0u << DRV83XX_OCP_OCPDEG_SHIFT)
#  define DRV83XX_OCP_OCPDEG_4US        (0x1u << DRV83XX_OCP_OCPDEG_SHIFT)
#  define DRV83XX_OCP_OCPDEG_6US        (0x2u << DRV83XX_OCP_OCPDEG_SHIFT)
#  define DRV83XX_OCP_OCPDEG_8US        (0x3u << DRV83XX_OCP_OCPDEG_SHIFT)
#define DRV83XX_OCP_OCPDEG(x)           (((x) << DRV83XX_OCP_OCPDEG_SHIFT) & \
                                         DRV83XX_OCP_OCPDEG_MASK)
#define DRV83XX_OCP_OCPMODE_SHIFT       (6)
#define DRV83XX_OCP_OCPMODE_MASK        (0x3u << DRV83XX_OCP_OCPMODE_SHIFT)
#  define DRV83XX_OCP_OCPMODE_LATCH     (0x0u << DRV83XX_OCP_OCPMODE_SHIFT)
#  define DRV83XX_OCP_OCPMODE_RETRY     (0x1u << DRV83XX_OCP_OCPMODE_SHIFT)
#  define DRV83XX_OCP_OCPMODE_REPORT    (0x2u << DRV83XX_OCP_OCPMODE_SHIFT)
#  define DRV83XX_OCP_OCPMODE_DISABLED  (0x3u << DRV83XX_OCP_OCPMODE_SHIFT)
#define DRV83XX_OCP_OCPMODE(x)          (((x) << DRV83XX_OCP_OCPMODE_SHIFT) & \
                                         DRV83XX_OCP_OCPMODE_MASK)
#define DRV83XX_OCP_DEADTIME_SHIFT      (8)
#define DRV83XX_OCP_DEADTIME_MASK       (0x3u << DRV83XX_OCP_DEADTIME_SHIFT)
#  define DRV83XX_OCP_DEADTIME_50NS     (0x0u << DRV83XX_OCP_DEADTIME_SHIFT)
#  define DRV83XX_OCP_DEADTIME_100NS    (0x1u << DRV83XX_OCP_DEADTIME_SHIFT)
#  define DRV83XX_OCP_DEADTIME_200NS    (0x2u << DRV83XX_OCP_DEADTIME_SHIFT)
#  define DRV83XX_OCP_DEADTIME_400NS    (0x3u << DRV83XX_OCP_DEADTIME_SHIFT)
#define DRV83XX_OCP_DEADTIME(x)         (((x) << DRV83XX_OCP_DEADTIME_SHIFT) & \
                                         DRV83XX_OCP_DEADTIME_MASK)
#define DRV83XX_OCP_TRETRY              (1u << 10) /* 0 = 4 ms, 1 = 50 us */

/* CSA Control Register (0x06, R/W) */

#define DRV83XX_CSACTL_SENLVL_SHIFT     (0)
#define DRV83XX_CSACTL_SENLVL_MASK      (0x3u << DRV83XX_CSACTL_SENLVL_SHIFT)
#  define DRV83XX_CSACTL_SENLVL_0V25    (0x0u << DRV83XX_CSACTL_SENLVL_SHIFT)
#  define DRV83XX_CSACTL_SENLVL_0V50    (0x1u << DRV83XX_CSACTL_SENLVL_SHIFT)
#  define DRV83XX_CSACTL_SENLVL_0V75    (0x2u << DRV83XX_CSACTL_SENLVL_SHIFT)
#  define DRV83XX_CSACTL_SENLVL_1V00    (0x3u << DRV83XX_CSACTL_SENLVL_SHIFT)
#define DRV83XX_CSACTL_SENLVL(x)        (((x) << DRV83XX_CSACTL_SENLVL_SHIFT) & \
                                         DRV83XX_CSACTL_SENLVL_MASK)
#define DRV83XX_CSACTL_CSA_CAL_C        (1u << 2)
#define DRV83XX_CSACTL_CSA_CAL_B        (1u << 3)
#define DRV83XX_CSACTL_CSA_CAL_A        (1u << 4)
#define DRV83XX_CSACTL_CSA_CAL_ALL      (DRV83XX_CSACTL_CSA_CAL_A | \
                                         DRV83XX_CSACTL_CSA_CAL_B | \
                                         DRV83XX_CSACTL_CSA_CAL_C)
#define DRV83XX_CSACTL_DIS_SEN          (1u << 5)
#define DRV83XX_CSACTL_GAIN_SHIFT       (6)
#define DRV83XX_CSACTL_GAIN_MASK        (0x3u << DRV83XX_CSACTL_GAIN_SHIFT)
#  define DRV83XX_CSACTL_GAIN_5         (0x0u << DRV83XX_CSACTL_GAIN_SHIFT)
#  define DRV83XX_CSACTL_GAIN_10        (0x1u << DRV83XX_CSACTL_GAIN_SHIFT)
#  define DRV83XX_CSACTL_GAIN_20        (0x2u << DRV83XX_CSACTL_GAIN_SHIFT)
#  define DRV83XX_CSACTL_GAIN_40        (0x3u << DRV83XX_CSACTL_GAIN_SHIFT)
#define DRV83XX_CSACTL_GAIN(x)          (((x) << DRV83XX_CSACTL_GAIN_SHIFT) & \
                                         DRV83XX_CSACTL_GAIN_MASK)
#define DRV83XX_CSACTL_LSREF            (1u << 8)
#define DRV83XX_CSACTL_VREFDIV          (1u << 9)
#define DRV83XX_CSACTL_CSA_FET          (1u << 10)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward declaration to avoid a hard dependency on <nuttx/spi/spi.h> in
 * users that only need register/bit definitions.
 */

struct spi_dev_s;

/* Chip variant.  The core today handles every member of the DRV832x family
 * that follows the standard 16-bit single-frame SPI protocol (DRV8323S,
 * DRV8320S, DRV8353S, ...).  Future chips with different framing (e.g.
 * DRV8301's CS-toggle between command and data) can be added here.
 */

enum drv83xx_chip_e
{
  DRV83XX_CHIP_DRV8323 = 0,
};

/* PWM input mode (DRIVER_CONTROL.PWM_MODE) */

enum drv83xx_pwm_mode_e
{
  DRV83XX_PWM_6X          = 0,
  DRV83XX_PWM_3X          = 1,
  DRV83XX_PWM_1X          = 2,
  DRV83XX_PWM_INDEPENDENT = 3,
};

/* Current sense amplifier gain (CSA_CONTROL.CSA_GAIN) */

enum drv83xx_csa_gain_e
{
  DRV83XX_GAIN_5  = 0,
  DRV83XX_GAIN_10 = 1,
  DRV83XX_GAIN_20 = 2,
  DRV83XX_GAIN_40 = 3,
};

/* IDRIVE source levels (used for IDRIVEP_HS and IDRIVEP_LS) */

enum drv83xx_idrivep_e
{
  DRV83XX_IDRIVEP_10MA   = 0x0,
  DRV83XX_IDRIVEP_30MA   = 0x1,
  DRV83XX_IDRIVEP_60MA   = 0x2,
  DRV83XX_IDRIVEP_80MA   = 0x3,
  DRV83XX_IDRIVEP_120MA  = 0x4,
  DRV83XX_IDRIVEP_140MA  = 0x5,
  DRV83XX_IDRIVEP_170MA  = 0x6,
  DRV83XX_IDRIVEP_190MA  = 0x7,
  DRV83XX_IDRIVEP_260MA  = 0x8,
  DRV83XX_IDRIVEP_330MA  = 0x9,
  DRV83XX_IDRIVEP_370MA  = 0xa,
  DRV83XX_IDRIVEP_440MA  = 0xb,
  DRV83XX_IDRIVEP_570MA  = 0xc,
  DRV83XX_IDRIVEP_680MA  = 0xd,
  DRV83XX_IDRIVEP_820MA  = 0xe,
  DRV83XX_IDRIVEP_1000MA = 0xf,
};

/* IDRIVE sink levels (used for IDRIVEN_HS and IDRIVEN_LS) */

enum drv83xx_idriven_e
{
  DRV83XX_IDRIVEN_20MA   = 0x0,
  DRV83XX_IDRIVEN_60MA   = 0x1,
  DRV83XX_IDRIVEN_120MA  = 0x2,
  DRV83XX_IDRIVEN_160MA  = 0x3,
  DRV83XX_IDRIVEN_240MA  = 0x4,
  DRV83XX_IDRIVEN_280MA  = 0x5,
  DRV83XX_IDRIVEN_340MA  = 0x6,
  DRV83XX_IDRIVEN_380MA  = 0x7,
  DRV83XX_IDRIVEN_520MA  = 0x8,
  DRV83XX_IDRIVEN_660MA  = 0x9,
  DRV83XX_IDRIVEN_740MA  = 0xa,
  DRV83XX_IDRIVEN_880MA  = 0xb,
  DRV83XX_IDRIVEN_1140MA = 0xc,
  DRV83XX_IDRIVEN_1360MA = 0xd,
  DRV83XX_IDRIVEN_1640MA = 0xe,
  DRV83XX_IDRIVEN_2000MA = 0xf,
};

/* TDRIVE peak gate-current drive time */

enum drv83xx_tdrive_e
{
  DRV83XX_TDRIVE_500NS = 0,
  DRV83XX_TDRIVE_1US   = 1,
  DRV83XX_TDRIVE_2US   = 2,
  DRV83XX_TDRIVE_4US   = 3,
};

/* Dead time between HS and LS transitions */

enum drv83xx_dead_time_e
{
  DRV83XX_DEAD_50NS  = 0,
  DRV83XX_DEAD_100NS = 1,
  DRV83XX_DEAD_200NS = 2,
  DRV83XX_DEAD_400NS = 3,
};

/* OCP response mode */

enum drv83xx_ocp_mode_e
{
  DRV83XX_OCP_LATCH    = 0,
  DRV83XX_OCP_RETRY    = 1,
  DRV83XX_OCP_REPORT   = 2,
  DRV83XX_OCP_DISABLED = 3,
};

/* OCP deglitch time */

enum drv83xx_ocp_deg_e
{
  DRV83XX_OCPDEG_2US = 0,
  DRV83XX_OCPDEG_4US = 1,
  DRV83XX_OCPDEG_6US = 2,
  DRV83XX_OCPDEG_8US = 3,
};

/* VDS overcurrent trip level */

enum drv83xx_vds_lvl_e
{
  DRV83XX_VDSLVL_0V06 = 0x0,
  DRV83XX_VDSLVL_0V13 = 0x1,
  DRV83XX_VDSLVL_0V20 = 0x2,
  DRV83XX_VDSLVL_0V26 = 0x3,
  DRV83XX_VDSLVL_0V31 = 0x4,
  DRV83XX_VDSLVL_0V45 = 0x5,
  DRV83XX_VDSLVL_0V53 = 0x6,
  DRV83XX_VDSLVL_0V60 = 0x7,
  DRV83XX_VDSLVL_0V68 = 0x8,
  DRV83XX_VDSLVL_0V75 = 0x9,
  DRV83XX_VDSLVL_0V94 = 0xa,
  DRV83XX_VDSLVL_1V13 = 0xb,
  DRV83XX_VDSLVL_1V30 = 0xc,
  DRV83XX_VDSLVL_1V50 = 0xd,
  DRV83XX_VDSLVL_1V70 = 0xe,
  DRV83XX_VDSLVL_1V88 = 0xf,
};

/* Sense overcurrent trip level (SP-pin) */

enum drv83xx_sen_lvl_e
{
  DRV83XX_SENLVL_0V25 = 0,
  DRV83XX_SENLVL_0V50 = 1,
  DRV83XX_SENLVL_0V75 = 2,
  DRV83XX_SENLVL_1V00 = 3,
};

/* Cold-start configuration.
 *
 * Each field maps directly to a bit-field of one of the R/W registers
 * 0x02..0x06 and is written verbatim by drv83xx_apply_cfg().
 */

struct drv83xx_cfg_s
{
  uint32_t freq;                /* SPI clock frequency [Hz] */

  /* DRIVER_CONTROL (0x02) */

  uint8_t  pwm_mode    : 2;     /* drv83xx_pwm_mode_e */
  uint8_t  dis_cpuv    : 1;     /* 1 = disable VCP UVLO fault */
  uint8_t  dis_gdf     : 1;     /* 1 = disable gate-drive fault */
  uint8_t  otw_rep     : 1;     /* 1 = report OTW on nFAULT */
  uint8_t  onepwm_com  : 1;     /* 1 = 1xPWM asynchronous rectification */
  uint8_t  onepwm_dir  : 1;     /* 1xPWM commutation direction bit */

  /* GATE_DRIVE_HS / LS (0x03 / 0x04) */

  uint8_t  idrivep_hs  : 4;     /* drv83xx_idrivep_e */
  uint8_t  idriven_hs  : 4;     /* drv83xx_idriven_e */
  uint8_t  idrivep_ls  : 4;     /* drv83xx_idrivep_e */
  uint8_t  idriven_ls  : 4;     /* drv83xx_idriven_e */
  uint8_t  tdrive      : 2;     /* drv83xx_tdrive_e */
  uint8_t  cbc         : 1;     /* 1 = cycle-by-cycle OC retry */

  /* OCP_CONTROL (0x05) */

  uint8_t  tretry      : 1;     /* 0 = 4 ms retry, 1 = 50 us */
  uint8_t  dead_time   : 2;     /* drv83xx_dead_time_e */
  uint8_t  ocp_mode    : 2;     /* drv83xx_ocp_mode_e */
  uint8_t  ocp_deg     : 2;     /* drv83xx_ocp_deg_e */
  uint8_t  vds_lvl     : 4;     /* drv83xx_vds_lvl_e */

  /* CSA_CONTROL (0x06) */

  uint8_t  csa_fet     : 1;     /* 1 = CSA+ on SHx (instead of SPx) */
  uint8_t  vref_div    : 1;     /* 1 = VREF/2 (bidirectional) */
  uint8_t  ls_ref      : 1;     /* 1 = LS VDS_OCP measured SHx-SNx */
  uint8_t  csa_gain    : 2;     /* drv83xx_csa_gain_e */
  uint8_t  dis_sen     : 1;     /* 1 = disable sense OC fault */
  uint8_t  sen_lvl     : 2;     /* drv83xx_sen_lvl_e */
};

/* Raw fault snapshot returned by drv83xx_get_faults() */

struct drv83xx_faults_s
{
  uint16_t fs1;                 /* raw FAULT_STATUS1 (DRV83XX_FS1_*) */
  uint16_t fs2;                 /* raw VGS_STATUS2   (DRV83XX_FS2_*) */
};

/* IDRIVE bundle for drv83xx_set_idrive() / drv83xx_get_idrive() */

struct drv83xx_idrive_s
{
  uint8_t idrivep_hs;           /* drv83xx_idrivep_e */
  uint8_t idriven_hs;           /* drv83xx_idriven_e */
  uint8_t idrivep_ls;           /* drv83xx_idrivep_e */
  uint8_t idriven_ls;           /* drv83xx_idriven_e */
};

/* CSA per-phase calibration request */

struct drv83xx_csa_cal_s
{
  bool ch_a;                    /* 1 = short CSA A inputs for offset cal */
  bool ch_b;
  bool ch_c;
};

/* Chip-agnostic device handle.
 *
 * The owner allocates this in its own private struct (typically embedded
 * as a member, not the first one) and passes a pointer in to every core
 * call.  The core is intentionally header-friendly so the FOC adapter,
 * the BLDC adapter and even userland self-tests can all share the exact
 * same SPI/register layer without dragging in foc_dev_s.
 */

struct drv83xx_dev_s
{
  FAR struct spi_dev_s *spi;   /* SPI bus this chip lives on */
  int                   devno; /* SPIDEV_MOTOR(devno) for chip-select */
  uint32_t              freq;  /* SPI clock frequency [Hz] */
  uint8_t               chip;  /* drv83xx_chip_e */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: drv83xx_init
 *
 * Description:
 *   Initialise a drv83xx_dev_s handle.  Does not touch hardware - merely
 *   stores the SPI/devno/freq/chip tuple for later use by the rest of the
 *   API.
 *
 ****************************************************************************/

void drv83xx_init(FAR struct drv83xx_dev_s *dev,
                  FAR struct spi_dev_s *spi,
                  int devno, uint32_t freq,
                  enum drv83xx_chip_e chip);

/****************************************************************************
 * Name: drv83xx_read_reg / drv83xx_write_reg / drv83xx_modify_reg
 *
 * Description:
 *   Raw 11-bit register access.  All three take the bus lock for the
 *   duration of the frame(s) and use SPIDEV_MOTOR(devno) for chip-select.
 *
 ****************************************************************************/

void drv83xx_read_reg(FAR struct drv83xx_dev_s *dev,
                      uint8_t addr, FAR uint16_t *data);

void drv83xx_write_reg(FAR struct drv83xx_dev_s *dev,
                       uint8_t addr, uint16_t data);

void drv83xx_modify_reg(FAR struct drv83xx_dev_s *dev,
                        uint8_t addr, uint16_t clr, uint16_t set);

/****************************************************************************
 * Name: drv83xx_apply_cfg
 *
 * Description:
 *   Program registers 0x02..0x06 from the supplied configuration in one
 *   shot, after first draining the read-only fault status registers and
 *   writing DRIVER_CONTROL with CLR_FLT set.  Caller is responsible for
 *   driving EN_GATE high and waiting tWAKE before invoking this.
 *
 ****************************************************************************/

int drv83xx_apply_cfg(FAR struct drv83xx_dev_s *dev,
                      FAR const struct drv83xx_cfg_s *cfg);

/****************************************************************************
 * Name: drv83xx_get_faults / drv83xx_clear_faults
 ****************************************************************************/

int drv83xx_get_faults(FAR struct drv83xx_dev_s *dev,
                       FAR struct drv83xx_faults_s *out);

int drv83xx_clear_faults(FAR struct drv83xx_dev_s *dev);

/****************************************************************************
 * Name: drv83xx_set_gain / drv83xx_get_gain
 *
 * Description:
 *   Map between the integer CSA gain (5/10/20/40) and the GAIN bit-field.
 *
 ****************************************************************************/

int drv83xx_set_gain(FAR struct drv83xx_dev_s *dev, int gain);
int drv83xx_get_gain(FAR struct drv83xx_dev_s *dev, FAR int *gain);

/****************************************************************************
 * Name: drv83xx_set_idrive / drv83xx_get_idrive
 ****************************************************************************/

int drv83xx_set_idrive(FAR struct drv83xx_dev_s *dev,
                       FAR const struct drv83xx_idrive_s *id);
int drv83xx_get_idrive(FAR struct drv83xx_dev_s *dev,
                       FAR struct drv83xx_idrive_s *id);

/****************************************************************************
 * Name: drv83xx_set_vds_lvl / drv83xx_get_vds_lvl
 ****************************************************************************/

int drv83xx_set_vds_lvl(FAR struct drv83xx_dev_s *dev, int lvl);
int drv83xx_get_vds_lvl(FAR struct drv83xx_dev_s *dev, FAR int *lvl);

/****************************************************************************
 * Name: drv83xx_set_ocp_mode / drv83xx_get_ocp_mode
 ****************************************************************************/

int drv83xx_set_ocp_mode(FAR struct drv83xx_dev_s *dev, int mode);
int drv83xx_get_ocp_mode(FAR struct drv83xx_dev_s *dev, FAR int *mode);

/****************************************************************************
 * Name: drv83xx_csa_phase_cal
 ****************************************************************************/

int drv83xx_csa_phase_cal(FAR struct drv83xx_dev_s *dev,
                          FAR const struct drv83xx_csa_cal_s *cal);

/****************************************************************************
 * Name: drv83xx_calibration_all
 *
 * Description:
 *   Convenience: short all three CSA inputs (state == true) to capture
 *   their offsets, or release them (state == false).  Matches the existing
 *   focpwr_ops_s::calibration contract.
 *
 ****************************************************************************/

int drv83xx_calibration_all(FAR struct drv83xx_dev_s *dev, bool state);

/****************************************************************************
 * Name: drv83xx_lock_regs
 *
 * Description:
 *   Lock (lock != 0) or unlock the gate-drive HS/LS configuration
 *   registers.  Writes are otherwise rejected by the device when locked.
 *
 ****************************************************************************/

int drv83xx_lock_regs(FAR struct drv83xx_dev_s *dev, int lock);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MOTOR_GATE_DRIVER_DRV83XX_CORE_H */
