/****************************************************************************
 * include/nuttx/motor/bldc/bldc_debug.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_BLDC_BLDC_DEBUG_H
#define __INCLUDE_NUTTX_MOTOR_BLDC_BLDC_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/motor/bldc/bldc.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_MOTOR_BLDC_DEBUG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLDCIOC_DEBUG_PWM_DUTIES       _MTRIOC(BLDC_IOC_BASE + 1)
#define BLDCIOC_DEBUG_PWM_OUTPUT       _MTRIOC(BLDC_IOC_BASE + 2)
#define BLDCIOC_DEBUG_PWM_INFO         _MTRIOC(BLDC_IOC_BASE + 3)
#define BLDCIOC_DEBUG_PWM_REGDUMP      _MTRIOC(BLDC_IOC_BASE + 4)

#define BLDCIOC_DEBUG_DRV_DUMP         _MTRIOC(BLDC_IOC_BASE + 10)

#define BLDCIOC_DEBUG_ADC_DUMP         _MTRIOC(BLDC_IOC_BASE + 20)
#define BLDCIOC_DEBUG_BEMF_STATS       _MTRIOC(BLDC_IOC_BASE + 21)

#define BLDCIOC_DEBUG_TRAP_RUN         _MTRIOC(BLDC_IOC_BASE + 30)
#define BLDCIOC_DEBUG_TRAP_STOP        _MTRIOC(BLDC_IOC_BASE + 31)
#define BLDCIOC_DEBUG_TRAP_SECTOR      _MTRIOC(BLDC_IOC_BASE + 32)
#define BLDCIOC_DEBUG_TRAP_STATUS      _MTRIOC(BLDC_IOC_BASE + 33)

#define BLDC_DEBUG_ADC_CHANNELS        4

/* Opaque word counts for board-specific register snapshots.  Board glue
 * defines the layout encoded in data[].
 */

#define BLDC_DEBUG_PWM_REGDUMP_WORDS   20
#define BLDC_DEBUG_DRV_DUMP_WORDS       8

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bldc_debug_pwm_duties_s
{
  uint16_t duty_a;
  uint16_t duty_b;
  uint16_t duty_c;
};

struct bldc_debug_pwm_output_s
{
  bool     enable;
  bool     brake;
};

struct bldc_debug_pwm_regdump_s
{
  uint32_t data[BLDC_DEBUG_PWM_REGDUMP_WORDS];
};

struct bldc_debug_drv_dump_s
{
  uint32_t data[BLDC_DEBUG_DRV_DUMP_WORDS];
};

struct bldc_debug_adc_dump_s
{
  uint16_t raw[BLDC_DEBUG_ADC_CHANNELS];
  uint8_t  channel[BLDC_DEBUG_ADC_CHANNELS];
  uint8_t  resolution;
  uint16_t vref_mv;
  uint32_t sample_count;
  uint32_t samples_dropped;
};

struct bldc_debug_bemf_stats_s
{
  uint32_t cmp_count[3];
  uint32_t stream_dropped_frames;
};

struct bldc_debug_trap_run_s
{
  uint16_t duty;
  uint32_t freq_electrical_hz;
  int8_t   direction;
};

struct bldc_debug_trap_sector_s
{
  uint8_t  sector;
  uint16_t duty;
};

struct bldc_debug_trap_status_s
{
  bool     running;
  uint8_t  state;
  uint8_t  sector;
  int8_t   direction;
  uint16_t duty;
  uint16_t duty_max;
  uint32_t freq_electrical_hz;
  uint32_t isr_ticks_per_sector;
  uint32_t commutations;
};

#endif /* CONFIG_MOTOR_BLDC_DEBUG */
#endif /* __INCLUDE_NUTTX_MOTOR_BLDC_BLDC_DEBUG_H */
