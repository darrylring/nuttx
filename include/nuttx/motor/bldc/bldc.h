/****************************************************************************
 * include/nuttx/motor/bldc/bldc.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_BLDC_BLDC_H
#define __INCLUDE_NUTTX_MOTOR_BLDC_BLDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/motor/motor.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_MOTOR_BLDC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BLDC ioctls.  Must not clash with MTRIOC_* or board-specific ranges. */

#define BLDC_IOC_BASE                  (200)

#define BLDCIOC_TRAP_SENSORLESS_CFG_SET   _MTRIOC(BLDC_IOC_BASE + 40)
#define BLDCIOC_TRAP_SENSORLESS_CFG_GET   _MTRIOC(BLDC_IOC_BASE + 41)
#define BLDCIOC_TRAP_SENSORLESS_STATE_GET _MTRIOC(BLDC_IOC_BASE + 42)
#define BLDCIOC_TRAP_SENSORLESS_DRIVE     _MTRIOC(BLDC_IOC_BASE + 43)

enum bldc_phase_state_e
{
  BLDC_PHASE_OFF = 0,
  BLDC_PHASE_LS  = 1,
  BLDC_PHASE_PWM = 2,
};

enum bldc_phase
{
  BLDC_PHASE_A = 0,
  BLDC_PHASE_B = 1,
  BLDC_PHASE_C = 2,
};

#define BLDC_DIR_FORWARD               (+1)
#define BLDC_DIR_REVERSE               (-1)
#define BLDC_TRAP_NSECTORS             6

enum bldc_trap_state_e
{
  BLDC_TRAP_STATE_IDLE   = 0,
  BLDC_TRAP_STATE_FIXED  = 1,
  BLDC_TRAP_STATE_CREEP  = 2,
  BLDC_TRAP_STATE_CLOSED = 3,
  BLDC_TRAP_STATE_STALL  = 4,
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bldc_pwm_info_s
{
  uint32_t frequency;
  uint16_t period;
  uint16_t deadtime;
};

struct bldc_commutation_s
{
  uint8_t  state_a;
  uint8_t  state_b;
  uint8_t  state_c;
  uint16_t duty;
};

struct bldc_adc_samples_s
{
  uint16_t v_phase_a;
  uint16_t v_phase_b;
  uint16_t v_phase_c;
  uint16_t v_bus;
};

struct bldc_trap_sensorless_cfg_s
{
  uint8_t  pole_pairs;
  uint16_t min_rps_electrical;
  int16_t  boost_decay_x1000;
  int16_t  speed_kp_x1000;
  int16_t  boost_kp_x1000;
  uint16_t stall_var_threshold_deg2;
  int16_t  phase_advance_mdeg;
  int16_t  phase_advance_speed_x1000;
};

struct bldc_trap_sensorless_state_s
{
  uint8_t  state;
  uint8_t  sector;
  int8_t   direction;
  bool     stalled;
  int16_t  amplitude_q15;
  int32_t  mech_rps_q16;
  int16_t  phase_error_deg;
  uint16_t phase_err_var_deg2;
  uint32_t commutations;
};

struct bldc_trap_sensorless_drive_s
{
  int16_t  amplitude_q15;
};

#ifdef CONFIG_MOTOR_BLDC_DEBUG

struct bldc_debug_pwm_regdump_s;
struct bldc_debug_drv_dump_s;
struct bldc_debug_adc_dump_s;
struct bldc_debug_bemf_stats_s;
#endif

struct bldc_dev_s;

/* Board and backend setup()/shutdown() are called from the motor char driver
 * on first open and last close (motor upper-half open count).
 *
 * The trapezoidal backend keeps commutation state across close (shutdown is
 * a no-op) so NSH builtins can open, ioctl, and close without stopping the
 * motor.  Use the trap stop ioctls or br_pwm motor stop / off to halt.
 *
 * Board shutdown hooks should likewise avoid disarming hardware that is
 * expensive to bring back (PWM, gate driver, ADC).
 */

struct bldc_board_ops_s
{
  CODE int (*setup)(FAR struct bldc_dev_s *dev);
  CODE int (*shutdown)(FAR struct bldc_dev_s *dev);

  CODE int (*pwm_get_info)(FAR struct bldc_dev_s *dev,
                           FAR struct bldc_pwm_info_s *info);

  CODE int (*commutation_apply)(FAR struct bldc_dev_s *dev,
                                FAR const struct bldc_commutation_s *cmd);

  CODE int (*fault_clear)(FAR struct bldc_dev_s *dev);

  CODE int (*ioctl)(FAR struct bldc_dev_s *dev, int cmd,
                    unsigned long arg);

#ifdef CONFIG_MOTOR_BLDC_DEBUG

  CODE int (*pwm_enable)(FAR struct bldc_dev_s *dev, bool enable);
  CODE int (*pwm_set_duties)(FAR struct bldc_dev_s *dev,
                             uint16_t duty_a,
                             uint16_t duty_b,
                             uint16_t duty_c);
  CODE int (*pwm_get_regdump)(FAR struct bldc_dev_s *dev,
                              FAR struct bldc_debug_pwm_regdump_s *r);
  CODE int (*drv_get_regdump)(FAR struct bldc_dev_s *dev,
                              FAR struct bldc_debug_drv_dump_s *d);
  CODE int (*adc_get_snapshot)(FAR struct bldc_dev_s *dev,
                               FAR struct bldc_debug_adc_dump_s *d);
  CODE int (*bemf_get_stats)(FAR struct bldc_dev_s *dev,
                             FAR struct bldc_debug_bemf_stats_s *s);
#endif
};

struct bldc_backend_ops_s
{
  FAR const char *name;
  uint8_t control_mode;
  uint32_t capabilities;

  CODE int (*setup)(FAR struct bldc_dev_s *dev);
  CODE int (*shutdown)(FAR struct bldc_dev_s *dev);
  CODE int (*start)(FAR struct bldc_dev_s *dev);
  CODE int (*stop)(FAR struct bldc_dev_s *dev);

  CODE int (*params_set)(FAR struct bldc_dev_s *dev,
                         FAR const struct motor_params_s *params);
  CODE int (*limits_set)(FAR struct bldc_dev_s *dev,
                         FAR const struct motor_limits_s *limits);
  CODE int (*mode_set)(FAR struct bldc_dev_s *dev, uint8_t mode);

  CODE int (*state_get)(FAR struct bldc_dev_s *dev,
                        FAR struct motor_state_s *state);

  CODE int (*fault_set)(FAR struct bldc_dev_s *dev, uint8_t fault);
  CODE int (*fault_get)(FAR struct bldc_dev_s *dev,
                        FAR uint8_t *fault);
  CODE int (*fault_clear)(FAR struct bldc_dev_s *dev, uint8_t fault);

  CODE int (*ioctl)(FAR struct bldc_dev_s *dev, int cmd,
                    unsigned long arg);

  CODE void (*commutation_tick)(FAR struct bldc_dev_s *dev,
                                FAR const struct bldc_adc_samples_s *adc);
};

struct bldc_dev_s
{
  struct motor_lowerhalf_s            lower;

  FAR const struct bldc_board_ops_s   *board;
  FAR void                            *board_priv;

  FAR const struct bldc_backend_ops_s *backend;
  FAR void                            *backend_priv;

  uint32_t                             capabilities;

  bool                                 running;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: bldc_register
 ****************************************************************************/

int bldc_register(FAR const char *path, FAR struct bldc_dev_s *dev);

/****************************************************************************
 * Name: bldc_commutation_tick
 ****************************************************************************/

static inline void
bldc_commutation_tick(FAR struct bldc_dev_s *dev,
                      FAR const struct bldc_adc_samples_s *adc)
{
  if (dev != NULL && dev->backend != NULL &&
      dev->backend->commutation_tick != NULL)
    {
      dev->backend->commutation_tick(dev, adc);
    }
}

/****************************************************************************
 * Backend factory helpers
 ****************************************************************************/

#ifdef CONFIG_MOTOR_BLDC_TRAP
FAR const struct bldc_backend_ops_s *bldc_trap_get_ops(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MOTOR_BLDC */
#endif /* __INCLUDE_NUTTX_MOTOR_BLDC_BLDC_H */
