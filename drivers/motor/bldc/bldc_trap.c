/****************************************************************************
 * drivers/motor/bldc/bldc_trap.c
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

/* Trapezoidal 6-step commutation backend (open-loop FIXED and sensorless
 * closed-loop CREEP / CLOSED / STALL).  commutation_tick() runs at the
 * board PWM-update rate with fresh phase-voltage samples for BEMF ZCD.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include <nuttx/motor/bldc/bldc.h>
#include <nuttx/motor/motor.h>

#ifdef CONFIG_MOTOR_BLDC_DEBUG
#  include <nuttx/motor/bldc/bldc_debug.h>
#endif

#include <assert.h>
#include <errno.h>
#include <math.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MOTOR_BLDC_TRAP_MAX_DUTY_PCT
#  define CONFIG_MOTOR_BLDC_TRAP_MAX_DUTY_PCT     25
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_POLE_PAIRS
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_POLE_PAIRS         7
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_MIN_RPS
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_MIN_RPS            20
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_BOOST_DECAY_X1000
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_BOOST_DECAY_X1000  950
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_SPEED_KP_X1000
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_SPEED_KP_X1000     1000
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_BOOST_KP_X1000
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_BOOST_KP_X1000     5000
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_STALL_VAR_DEG2
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_STALL_VAR_DEG2     150
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_PHASE_ADVANCE_MDEG
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_PHASE_ADVANCE_MDEG 0
#endif

#ifndef CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_PHASE_ADV_SPEED_X1000
#  define CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_PHASE_ADV_SPEED_X1000 730
#endif

#define BLDC_TRAP_SENSORLESS_CORR_CLAMP   2.5e-3f
#define BLDC_TRAP_PHASE_ERR_LPF_GAIN      0.1f
#define BLDC_TRAP_STALL_DEBOUNCE_TRIP        2000
#define BLDC_TRAP_STALL_RECOVER_LOCKOUT     (-5000)
#define BLDC_TRAP_STALL_HYSTERESIS_DEG2     50.0f
#define BLDC_TRAP_Q15_FULLSCALE              32767u
#define BLDC_TRAP_MIN_TICKS_PER_SECTOR    1u
#define BLDC_TRAP_MAX_TICKS_PER_SECTOR    0xFFFFFFFFu

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct trap_priv_s
{
  bool                  armed;
  bool                  running;
  uint8_t               state;

  volatile uint8_t      sector;
  volatile uint8_t      seg;
  int8_t                direction;
  volatile uint16_t     duty;
  uint16_t              duty_max;
  volatile uint32_t     commutations;

  volatile float        m_mech_revs;
  volatile float        m_mech_revs_per_tick;
  uint32_t              m_period_ticks;

  volatile float        zcd_acc_before;
  volatile float        zcd_acc_after;
  volatile uint8_t      zcd_last_seg;
  volatile bool         zcd_have_first;
  volatile float        phase_err_rev;
  volatile float        phase_err_deg;
  volatile float        phase_err_deg_mean;
  volatile float        phase_err_sq_mean;
  volatile float        phase_err_var_deg2;

  volatile int16_t      amp_q15;
  volatile bool         desired_cw;
  volatile bool         reset_speed;
  volatile float        mech_revs_rate_boost;
  volatile float        speed_correction;
  volatile float        speed_correction_lpf;
  volatile int32_t      stall_counter;
  volatile bool         stalled;

  struct bldc_trap_sensorless_cfg_s cfg;
  float                 min_revs_per_tick;

  uint32_t              pwm_freq;
  uint32_t              freq_hz;
};

static struct trap_priv_s g_default_trap;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define BLDC_TRAP_NSEGMENTS  12u

static const uint8_t g_commutation_lookup[BLDC_TRAP_NSEGMENTS] =
{
  BLDC_PHASE_OFF,
  BLDC_PHASE_PWM,
  BLDC_PHASE_PWM,
  BLDC_PHASE_PWM,
  BLDC_PHASE_PWM,
  BLDC_PHASE_OFF,
  BLDC_PHASE_OFF,
  BLDC_PHASE_LS,
  BLDC_PHASE_LS,
  BLDC_PHASE_LS,
  BLDC_PHASE_LS,
  BLDC_PHASE_OFF,
};

static const uint8_t g_undriven_lookup[BLDC_TRAP_NSEGMENTS] =
{
  BLDC_PHASE_A,
  BLDC_PHASE_C,
  BLDC_PHASE_C,
  BLDC_PHASE_B,
  BLDC_PHASE_B,
  BLDC_PHASE_A,
  BLDC_PHASE_A,
  BLDC_PHASE_C,
  BLDC_PHASE_C,
  BLDC_PHASE_B,
  BLDC_PHASE_B,
  BLDC_PHASE_A,
};

/* Map 6-step sector index to 12-segment table index. */

static inline uint8_t trap_seg_from_sector(uint8_t sector)
{
  return (uint8_t)(((uint32_t)sector * 2u + 2u) % BLDC_TRAP_NSEGMENTS);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR struct trap_priv_s *trap_priv(FAR struct bldc_dev_s *dev)
{
  return (dev->backend_priv != NULL)
         ? (FAR struct trap_priv_s *)dev->backend_priv
         : &g_default_trap;
}

/****************************************************************************
 * Name: trap_get_pwm_info
 ****************************************************************************/

static int trap_get_pwm_info(FAR struct bldc_dev_s *dev,
                              FAR uint32_t *pwm_freq,
                              FAR uint16_t *period)
{
  struct bldc_pwm_info_s info;
  int                    ret;

  if (dev->board == NULL || dev->board->pwm_get_info == NULL)
    {
      return -ENOSYS;
    }

  ret = dev->board->pwm_get_info(dev, &info);
  if (ret < 0)
    {
      return ret;
    }

  if (info.frequency == 0 || info.period == 0)
    {
      return -EINVAL;
    }

  *pwm_freq = info.frequency;
  *period   = info.period;
  return OK;
}

/****************************************************************************
 * Name: trap_fill_commutation_seg
 ****************************************************************************/

static void trap_fill_commutation_seg(FAR struct bldc_commutation_s *cmd,
                                      uint8_t seg, uint16_t duty)
{
  DEBUGASSERT(cmd != NULL);
  DEBUGASSERT(seg < BLDC_TRAP_NSEGMENTS);

  uint8_t seg_b = (uint8_t)((seg + 8u) % BLDC_TRAP_NSEGMENTS);
  uint8_t seg_c = (uint8_t)((seg + 4u) % BLDC_TRAP_NSEGMENTS);

  cmd->state_a = g_commutation_lookup[seg];
  cmd->state_b = g_commutation_lookup[seg_b];
  cmd->state_c = g_commutation_lookup[seg_c];
  cmd->duty    = duty;
}

/****************************************************************************
 * Name: apply_table_drive
 ****************************************************************************/

static int apply_table_drive(FAR struct bldc_dev_s *dev,
                             uint8_t seg, uint16_t duty)
{
  struct bldc_commutation_s cmd;

  if (dev->board == NULL || dev->board->commutation_apply == NULL)
    {
      return -ENOSYS;
    }

  trap_fill_commutation_seg(&cmd, seg, duty);
  return dev->board->commutation_apply(dev, &cmd);
}

/****************************************************************************
 * Name: trap_apply_safe
 ****************************************************************************/

static int trap_apply_safe(FAR struct bldc_dev_s *dev)
{
  struct bldc_commutation_s cmd =
  {
    .state_a = BLDC_PHASE_OFF,
    .state_b = BLDC_PHASE_OFF,
    .state_c = BLDC_PHASE_OFF,
    .duty    = 0,
  };

  if (dev->board == NULL || dev->board->commutation_apply == NULL)
    {
      return -ENOSYS;
    }

  return dev->board->commutation_apply(dev, &cmd);
}

/****************************************************************************
 * Name: trap_duty_max_from_period
 ****************************************************************************/

static uint16_t trap_duty_max_from_period(uint16_t period)
{
  uint32_t scaled;

  if (period == 0)
    {
      return 0;
    }

  scaled = (uint32_t)period * CONFIG_MOTOR_BLDC_TRAP_MAX_DUTY_PCT / 100;
  return (uint16_t)((scaled > period) ? period : scaled);
}

/****************************************************************************
 * Name: trap_compute_duty_max
 ****************************************************************************/

static int trap_compute_duty_max(FAR struct bldc_dev_s *dev,
                                  FAR uint16_t *duty_max)
{
  uint32_t pwm_freq;
  uint16_t period;
  int      ret;

  ret = trap_get_pwm_info(dev, &pwm_freq, &period);
  if (ret < 0)
    {
      return ret;
    }

  *duty_max = trap_duty_max_from_period(period);
  return OK;
}

/****************************************************************************
 * Name: trap_ticks_from_us / trap_ticks_from_ms
 ****************************************************************************/

unused_code static uint32_t trap_ticks_from_us(uint32_t pwm_freq, uint32_t us)
{
  return (uint32_t)(((uint64_t)us * pwm_freq + 500000u) / 1000000u);
}

unused_code static uint32_t trap_ticks_from_ms(uint32_t pwm_freq, uint32_t ms)
{
  return (uint32_t)(((uint64_t)ms * pwm_freq + 500u) / 1000u);
}

/****************************************************************************
 * Name: trap_compute_revs_per_tick
 ****************************************************************************/

static inline float trap_compute_revs_per_tick(uint32_t pwm_freq,
                                               uint32_t freq_hz,
                                               int8_t   direction)
{
  float v;

  if (pwm_freq == 0)
    {
      return 0.0f;
    }

  v = (float)freq_hz / (float)pwm_freq;
  return (direction < 0) ? -v : v;
}

/****************************************************************************
 * Name: trap_phase_to_period_ticks
 ****************************************************************************/

static inline uint32_t trap_phase_to_period_ticks(float revs_per_tick)
{
  float mag = (revs_per_tick < 0.0f) ? -revs_per_tick : revs_per_tick;

  if (mag <= 0.0f)
    {
      return BLDC_TRAP_MAX_TICKS_PER_SECTOR;
    }

  float ticks = 1.0f / (mag * (float)BLDC_TRAP_NSEGMENTS);

  if (ticks < (float)BLDC_TRAP_MIN_TICKS_PER_SECTOR)
    {
      return BLDC_TRAP_MIN_TICKS_PER_SECTOR;
    }

  if (ticks > (float)BLDC_TRAP_MAX_TICKS_PER_SECTOR)
    {
      return BLDC_TRAP_MAX_TICKS_PER_SECTOR;
    }

  return (uint32_t)ticks;
}


/****************************************************************************
 * Name: trap_set_sector
 ****************************************************************************/

static int trap_set_sector(FAR struct bldc_dev_s *dev,
                           uint8_t sector, uint16_t duty)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  uint8_t                 seg;

  if (sector >= BLDC_TRAP_NSECTORS)
    {
      return -EINVAL;
    }

  if (duty > priv->duty_max)
    {
      duty = priv->duty_max;
    }

  seg = trap_seg_from_sector(sector);

  priv->sector      = sector;
  priv->seg         = seg;
  priv->m_mech_revs = ((float)seg + 0.5f) / (float)BLDC_TRAP_NSEGMENTS;
  priv->duty        = duty;

  return apply_table_drive(dev, seg, duty);
}

/****************************************************************************
 * Name: trap_adc_get_phase
 ****************************************************************************/

static inline float trap_adc_get_phase(
    FAR const struct bldc_adc_samples_s *adc, uint8_t phase)
{
  switch (phase)
    {
      case BLDC_PHASE_A: return (float)adc->v_phase_a;
      case BLDC_PHASE_B: return (float)adc->v_phase_b;
      case BLDC_PHASE_C: return (float)adc->v_phase_c;
      default:           return 0.0f;
    }
}

/****************************************************************************
 * Name: compute_phase_err_from_adc
 ****************************************************************************/

static float compute_phase_err_from_adc(
    FAR struct trap_priv_s *priv,
    uint8_t seg, bool cw,
    FAR const struct bldc_adc_samples_s *adc)
{
  float result = NAN;
  uint8_t undriven_phase;
  uint8_t driven_phase_1;
  uint8_t driven_phase_2;
  float v_undriven;
  float v_thresh;
  float v_comp;
  bool  after_zc;
  bool  falling;

  if (seg != priv->zcd_last_seg)
    {
      bool prev_was_after_zc;

      prev_was_after_zc = ((priv->zcd_last_seg & 0x1u) == 0u);
      if (!cw)
        {
          prev_was_after_zc = !prev_was_after_zc;
        }

      if (prev_was_after_zc)
        {
          float num = priv->zcd_acc_before - priv->zcd_acc_after;
          float den = priv->zcd_acc_before + priv->zcd_acc_after;

          if (den > 0.0f)
            {
              result = (num / den) * (1.0f / 12.0f);
            }

          priv->zcd_acc_before = 0.0f;
          priv->zcd_acc_after  = 0.0f;
        }

      priv->zcd_last_seg = seg;
    }

  undriven_phase = g_undriven_lookup[seg];
  driven_phase_1 = g_undriven_lookup[(seg + 2u) % BLDC_TRAP_NSEGMENTS];
  driven_phase_2 = g_undriven_lookup[(seg + 4u) % BLDC_TRAP_NSEGMENTS];

  v_undriven = trap_adc_get_phase(adc, undriven_phase);
  v_thresh   = (1.0f / 3.0f) *
               (v_undriven +
                trap_adc_get_phase(adc, driven_phase_1) +
                trap_adc_get_phase(adc, driven_phase_2));
  v_comp     = v_undriven - v_thresh;

  after_zc = ((seg & 0x1u) == 0u);
  falling  = (((seg + 1u) & 0x2u) == 0x2u);

  if (falling)
    {
      v_comp = -v_comp;
    }

  if (!cw)
    {
      after_zc = !after_zc;
      v_comp   = -v_comp;
    }

  if (!after_zc)
    {
      v_comp = -v_comp;
    }

  v_comp = (v_comp > 0.0f) ? 1.0f : 0.0f;

  if (after_zc)
    {
      priv->zcd_acc_after  += v_comp;
    }
  else
    {
      priv->zcd_acc_before += v_comp;
    }

  return result;
}

/****************************************************************************
 * Name: trap_update_phase_err_stats
 ****************************************************************************/

static inline void trap_update_phase_err_stats(FAR struct trap_priv_s *priv,
                                               float err_deg)
{
  if (!priv->zcd_have_first)
    {
      priv->phase_err_deg_mean = err_deg;
      priv->phase_err_sq_mean  = err_deg * err_deg;
      priv->zcd_have_first     = true;
    }
  else
    {
      priv->phase_err_deg_mean +=
          (err_deg - priv->phase_err_deg_mean) *
          BLDC_TRAP_PHASE_ERR_LPF_GAIN;
      priv->phase_err_sq_mean  +=
          (err_deg * err_deg - priv->phase_err_sq_mean) *
          BLDC_TRAP_PHASE_ERR_LPF_GAIN;
    }

  priv->phase_err_var_deg2 = priv->phase_err_sq_mean -
                             priv->phase_err_deg_mean *
                             priv->phase_err_deg_mean;
  if (priv->phase_err_var_deg2 < 0.0f)
    {
      priv->phase_err_var_deg2 = 0.0f;
    }
}

/****************************************************************************
 * Name: trap_compute_min_revs_per_tick
 ****************************************************************************/

static inline float trap_compute_min_revs_per_tick(uint32_t pwm_freq,
                                                   uint16_t min_rps)
{
  if (pwm_freq == 0)
    {
      return 0.0f;
    }

  return (float)min_rps / (float)pwm_freq;
}

/****************************************************************************
 * Name: trap_load_default_cfg
 ****************************************************************************/

static void trap_load_default_cfg(FAR struct trap_priv_s *priv)
{
  priv->cfg.pole_pairs               =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_POLE_PAIRS;
  priv->cfg.min_rps_electrical       =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_MIN_RPS;
  priv->cfg.boost_decay_x1000        =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_BOOST_DECAY_X1000;
  priv->cfg.speed_kp_x1000           =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_SPEED_KP_X1000;
  priv->cfg.boost_kp_x1000           =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_BOOST_KP_X1000;
  priv->cfg.stall_var_threshold_deg2 =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_STALL_VAR_DEG2;
  priv->cfg.phase_advance_mdeg       =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_PHASE_ADVANCE_MDEG;
  priv->cfg.phase_advance_speed_x1000 =
      CONFIG_MOTOR_BLDC_TRAP_SENSORLESS_PHASE_ADV_SPEED_X1000;
}

/****************************************************************************
 * Name: trap_seed_creep_speed
 ****************************************************************************/

static inline void trap_seed_creep_speed(FAR struct trap_priv_s *priv,
                                         bool cw)
{
  float min_rev = priv->min_revs_per_tick;

  priv->m_mech_revs_per_tick = cw ?  min_rev : -min_rev;
  priv->mech_revs_rate_boost = 0.0f;
}

/****************************************************************************
 * Name: trap_apply_regulator
 ****************************************************************************/

static inline void trap_apply_regulator(FAR struct trap_priv_s *priv,
                                        float err_rev)
{
  float revs_per_tick = priv->m_mech_revs_per_tick;
  float abs_revs      = (revs_per_tick < 0.0f) ? -revs_per_tick
                                               :  revs_per_tick;
  bool  too_slow      = abs_revs < priv->min_revs_per_tick;

  if (too_slow && err_rev > 0.0f)
    {
      return;
    }

  float clamp_mag = (abs_revs < BLDC_TRAP_SENSORLESS_CORR_CLAMP)
                    ? abs_revs
                    : BLDC_TRAP_SENSORLESS_CORR_CLAMP;
  float dir_sign  = priv->desired_cw ? 1.0f : -1.0f;
  float corr      = -err_rev * clamp_mag;

  priv->speed_correction      = corr;
  priv->speed_correction_lpf += (corr - priv->speed_correction_lpf) * 1.0f;

  if (priv->amp_q15 != 0)
    {
      float kp_speed = (float)priv->cfg.speed_kp_x1000 * (1.0f / 1000.0f);
      float kp_boost = (float)priv->cfg.boost_kp_x1000 * (1.0f / 1000.0f);

      priv->m_mech_revs_per_tick += dir_sign *
                                    priv->speed_correction_lpf *
                                    kp_speed;
      priv->mech_revs_rate_boost  = dir_sign *
                                    priv->speed_correction_lpf *
                                    kp_boost;

      if (!priv->stalled)
        {
          priv->state = BLDC_TRAP_STATE_CLOSED;
        }
    }
  else
    {
      trap_seed_creep_speed(priv, priv->desired_cw);
    }
}

/****************************************************************************
 * Name: trap_update_stall_detection
 ****************************************************************************/

static inline void trap_update_stall_detection(FAR struct trap_priv_s *priv)
{
  float threshold = (float)priv->cfg.stall_var_threshold_deg2 +
                    (priv->stalled ?  BLDC_TRAP_STALL_HYSTERESIS_DEG2
                                   : -BLDC_TRAP_STALL_HYSTERESIS_DEG2);

  if (priv->phase_err_var_deg2 > threshold)
    {
      if (priv->stall_counter < BLDC_TRAP_STALL_DEBOUNCE_TRIP)
        {
          priv->stall_counter++;
        }
      else if (!priv->stalled)
        {
          priv->stalled = true;
          if (priv->state != BLDC_TRAP_STATE_FIXED)
            {
              priv->state = BLDC_TRAP_STATE_STALL;
            }
        }
    }
  else
    {
      if (priv->stall_counter > 0)
        {
          priv->stall_counter--;
        }
      else if (priv->stalled)
        {
          priv->stalled       = false;
          priv->stall_counter = BLDC_TRAP_STALL_RECOVER_LOCKOUT;
          if (priv->state == BLDC_TRAP_STATE_STALL)
            {
              priv->state = (priv->amp_q15 != 0) ?
                            BLDC_TRAP_STATE_CLOSED :
                            BLDC_TRAP_STATE_CREEP;
            }
        }
    }
}

/****************************************************************************
 * Name: trap_enforce_creep_floor
 ****************************************************************************/

static inline void trap_enforce_creep_floor(FAR struct trap_priv_s *priv)
{
  float abs_revs = (priv->m_mech_revs_per_tick < 0.0f)
                   ? -priv->m_mech_revs_per_tick
                   :  priv->m_mech_revs_per_tick;

  bool too_slow = abs_revs < priv->min_revs_per_tick;

  if (too_slow || priv->reset_speed)
    {
      trap_seed_creep_speed(priv, priv->desired_cw);
      priv->reset_speed = false;

      if (priv->state == BLDC_TRAP_STATE_CLOSED)
        {
          priv->state = BLDC_TRAP_STATE_CREEP;
        }
    }
}

/****************************************************************************
 * Name: trap_commutation_tick
 ****************************************************************************/

static void trap_commutation_tick(FAR struct bldc_dev_s *dev,
                                  FAR const struct bldc_adc_samples_s *adc)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  float                   phase;
  float                   boost_decay;
  uint8_t                 seg;
  bool                    closed_loop;

  if (!priv->running)
    {
      return;
    }

  closed_loop = (priv->state != BLDC_TRAP_STATE_FIXED);

  if (closed_loop)
    {
      trap_enforce_creep_floor(priv);
    }

  phase = priv->m_mech_revs + priv->m_mech_revs_per_tick +
          priv->mech_revs_rate_boost;

  if (closed_loop)
    {
      boost_decay = (float)priv->cfg.boost_decay_x1000 * (1.0f / 1000.0f);
      priv->mech_revs_rate_boost *= boost_decay;
    }
  else
    {
      priv->mech_revs_rate_boost = 0.0f;
    }

  phase -= floorf(phase);
  priv->m_mech_revs = phase;

  seg = (uint8_t)(phase * (float)BLDC_TRAP_NSEGMENTS);
  if (seg >= BLDC_TRAP_NSEGMENTS)
    {
      seg = BLDC_TRAP_NSEGMENTS - 1;
    }

  if (seg != priv->seg)
    {
      (void)apply_table_drive(dev, seg, priv->duty);
      priv->seg          = seg;
      priv->sector       = (uint8_t)((seg + BLDC_TRAP_NSEGMENTS -
                                      trap_seg_from_sector(0)) /
                                     2u % BLDC_TRAP_NSECTORS);
      priv->commutations++;
    }

  if (adc != NULL)
    {
      bool  cw      = (priv->m_mech_revs_per_tick >= 0.0f);
      float err_rev = compute_phase_err_from_adc(priv, seg, cw, adc);

      if (isfinite(err_rev))
        {
          float err_deg = err_rev * 360.0f;

          priv->phase_err_rev = err_rev;
          priv->phase_err_deg = err_deg;
          trap_update_phase_err_stats(priv, err_deg);

          if (closed_loop)
            {
              float static_rev =
                  (float)priv->cfg.phase_advance_mdeg *
                  (1.0f / (1000.0f * 360.0f));
              float speed_gain =
                  (float)priv->cfg.phase_advance_speed_x1000 *
                  (1.0f / 1000.0f);
              float dp = priv->m_mech_revs_per_tick * speed_gain +
                         static_rev;
              float adv_err = err_rev + dp;

              trap_update_stall_detection(priv);
              trap_apply_regulator(priv, adv_err);
            }
        }
    }
}

/****************************************************************************
 * Name: trap_start_running   (open-loop / FIXED)
 ****************************************************************************/

static int trap_start_running(FAR struct bldc_dev_s *dev,
                              uint16_t duty,
                              uint32_t freq_hz,
                              int8_t   direction)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  irqstate_t              flags;
  uint32_t                pwm_freq;
  uint16_t                period;
  int                     ret;

  if (!priv->armed)
    {
      return -EPERM;
    }

  if (dev->board == NULL || dev->board->commutation_apply == NULL)
    {
      return -ENOSYS;
    }

  ret = trap_get_pwm_info(dev, &pwm_freq, &period);
  if (ret < 0)
    {
      return ret;
    }

  priv->pwm_freq = pwm_freq;
  priv->duty_max = trap_duty_max_from_period(period);

  if (duty > priv->duty_max)
    {
      duty = priv->duty_max;
    }

  flags = enter_critical_section();
  priv->state                = BLDC_TRAP_STATE_FIXED;
  priv->direction            = (direction < 0) ? BLDC_DIR_REVERSE
                                               : BLDC_DIR_FORWARD;
  priv->duty                 = duty;
  priv->freq_hz              = freq_hz;
  priv->m_mech_revs_per_tick = trap_compute_revs_per_tick(pwm_freq,
                                                          freq_hz,
                                                          priv->direction);
  priv->m_period_ticks       = trap_phase_to_period_ticks(
                                  priv->m_mech_revs_per_tick);
  priv->sector               = 0;
  priv->seg                  = trap_seg_from_sector(0);
  priv->m_mech_revs          = ((float)priv->seg + 0.5f) /
                               (float)BLDC_TRAP_NSEGMENTS;
  priv->commutations         = 0;

  priv->zcd_acc_before       = 0.0f;
  priv->zcd_acc_after        = 0.0f;
  priv->zcd_last_seg         = priv->seg;
  priv->zcd_have_first       = false;
  priv->phase_err_rev        = 0.0f;
  priv->phase_err_deg        = 0.0f;
  priv->phase_err_deg_mean   = 0.0f;
  priv->phase_err_sq_mean    = 0.0f;
  priv->phase_err_var_deg2   = 0.0f;

  priv->running              = true;
  leave_critical_section(flags);

  ret = trap_set_sector(dev, 0, duty);
  if (ret < 0)
    {
      flags = enter_critical_section();
      priv->running = false;
      priv->state   = BLDC_TRAP_STATE_IDLE;
      leave_critical_section(flags);
      (void)trap_apply_safe(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: trap_start_closed_loop
 ****************************************************************************/

static int trap_start_closed_loop(FAR struct bldc_dev_s *dev,
                                  int16_t amplitude_q15)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  irqstate_t              flags;
  uint32_t                pwm_freq;
  uint16_t                period;
  uint16_t                duty;
  bool                    cw;
  int8_t                  direction;
  int                     ret;

  if (!priv->armed)
    {
      return -EPERM;
    }

  if (dev->board == NULL || dev->board->commutation_apply == NULL)
    {
      return -ENOSYS;
    }

  ret = trap_get_pwm_info(dev, &pwm_freq, &period);
  if (ret < 0)
    {
      return ret;
    }

  priv->pwm_freq          = pwm_freq;
  priv->duty_max          = trap_duty_max_from_period(period);
  priv->min_revs_per_tick = trap_compute_min_revs_per_tick(
                                pwm_freq, priv->cfg.min_rps_electrical);

  if (amplitude_q15 != 0)
    {
      uint32_t mag = (amplitude_q15 < 0) ? (uint32_t)(-amplitude_q15)
                                         : (uint32_t)amplitude_q15;
      uint32_t d   = (mag * priv->duty_max) / BLDC_TRAP_Q15_FULLSCALE;
      duty = (d > priv->duty_max) ? priv->duty_max : (uint16_t)d;
    }
  else
    {
      duty = 0;
    }

  cw        = (amplitude_q15 >= 0);
  direction = cw ? BLDC_DIR_FORWARD : BLDC_DIR_REVERSE;

  flags = enter_critical_section();

  priv->state                = BLDC_TRAP_STATE_CREEP;
  priv->direction            = direction;
  priv->desired_cw           = cw;
  priv->reset_speed          = true;
  priv->amp_q15              = amplitude_q15;
  priv->duty                 = duty;
  priv->freq_hz              = priv->cfg.min_rps_electrical;
  priv->m_mech_revs_per_tick = cw ?  priv->min_revs_per_tick
                                  : -priv->min_revs_per_tick;
  priv->mech_revs_rate_boost = 0.0f;
  priv->m_period_ticks       = trap_phase_to_period_ticks(
                                  priv->m_mech_revs_per_tick);
  priv->sector               = 0;
  priv->seg                  = trap_seg_from_sector(0);
  priv->m_mech_revs          = ((float)priv->seg + 0.5f) /
                               (float)BLDC_TRAP_NSEGMENTS;
  priv->commutations         = 0;

  priv->zcd_acc_before       = 0.0f;
  priv->zcd_acc_after        = 0.0f;
  priv->zcd_last_seg         = priv->seg;
  priv->zcd_have_first       = false;
  priv->phase_err_rev        = 0.0f;
  priv->phase_err_deg        = 0.0f;
  priv->phase_err_deg_mean   = 0.0f;
  priv->phase_err_sq_mean    = 0.0f;
  priv->phase_err_var_deg2   = 0.0f;
  priv->speed_correction     = 0.0f;
  priv->speed_correction_lpf = 0.0f;
  priv->stall_counter        = 0;
  priv->stalled              = false;

  priv->running              = true;
  leave_critical_section(flags);

  ret = trap_set_sector(dev, 0, duty);
  if (ret < 0)
    {
      flags = enter_critical_section();
      priv->running = false;
      priv->state   = BLDC_TRAP_STATE_IDLE;
      leave_critical_section(flags);
      (void)trap_apply_safe(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: trap_update_drive
 ****************************************************************************/

static int trap_update_drive(FAR struct bldc_dev_s *dev,
                             int16_t amplitude_q15)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  irqstate_t              flags;
  uint16_t                duty;
  bool                    cw;

  if (!priv->armed)
    {
      return -EPERM;
    }

  if (priv->state == BLDC_TRAP_STATE_FIXED)
    {
      return -EBUSY;
    }

  if (amplitude_q15 != 0)
    {
      uint32_t mag = (amplitude_q15 < 0) ? (uint32_t)(-amplitude_q15)
                                         : (uint32_t)amplitude_q15;
      uint32_t d   = (mag * priv->duty_max) / BLDC_TRAP_Q15_FULLSCALE;
      duty = (d > priv->duty_max) ? priv->duty_max : (uint16_t)d;
    }
  else
    {
      duty = 0;
    }

  cw = (amplitude_q15 >= 0);

  flags = enter_critical_section();
  if (cw != priv->desired_cw)
    {
      priv->reset_speed = true;
    }
  priv->desired_cw = cw;
  priv->direction  = cw ? BLDC_DIR_FORWARD : BLDC_DIR_REVERSE;
  priv->amp_q15    = amplitude_q15;
  priv->duty       = duty;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: trap_stop_running
 ****************************************************************************/

static int trap_stop_running(FAR struct bldc_dev_s *dev)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  irqstate_t              flags;

  flags = enter_critical_section();
  priv->running              = false;
  priv->state                = BLDC_TRAP_STATE_IDLE;
  priv->m_mech_revs_per_tick = 0.0f;
  priv->mech_revs_rate_boost = 0.0f;
  priv->amp_q15              = 0;
  priv->stalled              = false;
  priv->stall_counter        = 0;
  leave_critical_section(flags);

  return trap_apply_safe(dev);
}

#ifdef CONFIG_MOTOR_BLDC_DEBUG
/****************************************************************************
 * Name: trap_status_get
 ****************************************************************************/

static void trap_status_get(FAR struct bldc_dev_s *dev,
                            FAR struct bldc_debug_trap_status_s *s)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  irqstate_t              flags;

  DEBUGASSERT(s != NULL);

  flags = enter_critical_section();
  s->running              = priv->running;
  s->state                = priv->state;
  s->sector               = priv->sector;
  s->direction            = priv->direction;
  s->duty                 = priv->duty;
  s->duty_max             = priv->duty_max;
  s->freq_electrical_hz   = priv->freq_hz;
  s->isr_ticks_per_sector = priv->m_period_ticks;
  s->commutations         = priv->commutations;
  leave_critical_section(flags);
}
#endif /* CONFIG_MOTOR_BLDC_DEBUG */

/****************************************************************************
 * Name: trap_setup / trap_shutdown / trap_start / trap_stop
 ****************************************************************************/

static int trap_setup(FAR struct bldc_dev_s *dev)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  uint32_t                pwm_freq;
  uint16_t                period;
  int                     ret;

  if (priv->armed)
    {
      return OK;
    }

  memset(priv, 0, sizeof(*priv));
  priv->armed     = true;
  priv->state     = BLDC_TRAP_STATE_IDLE;
  priv->direction = BLDC_DIR_FORWARD;

  trap_load_default_cfg(priv);

  ret = trap_compute_duty_max(dev, &priv->duty_max);
  if (ret < 0)
    {
      priv->armed = false;
      return ret;
    }

  ret = trap_get_pwm_info(dev, &pwm_freq, &period);
  if (ret < 0)
    {
      priv->armed = false;
      return ret;
    }

  priv->pwm_freq          = pwm_freq;
  priv->min_revs_per_tick =
      trap_compute_min_revs_per_tick(pwm_freq,
                                     priv->cfg.min_rps_electrical);

  (void)trap_apply_safe(dev);
  return OK;
}

static int trap_shutdown(FAR struct bldc_dev_s *dev)
{
  /* Deliberate no-op for NSH-style use (open -> ioctl -> close per command).
   * br_pwm closes /dev/motor0 after every invocation; tearing the backend
   * down here would stop a running commutation and wipe trap state/cfg
   * before the next command.  Explicit stop: BLDCIOC_DEBUG_TRAP_STOP,
   * BLDCIOC_TRAP_SENSORLESS_DRIVE amp=0, or br_pwm motor stop / off.
   *
   * board_motor_shutdown() is also a no-op so TIM8/DRV/ADC stay armed.
   */

  (void)dev;
  return OK;
}

static int trap_start(FAR struct bldc_dev_s *dev)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);
  uint16_t                duty;
  uint32_t                freq_hz;

  if (!priv->armed)
    {
      return -EPERM;
    }

  duty    = (priv->duty    > 0) ? priv->duty    : (priv->duty_max / 2);
  freq_hz = (priv->freq_hz > 0) ? priv->freq_hz : 5u;

  return trap_start_running(dev, duty, freq_hz, priv->direction);
}

static int trap_stop(FAR struct bldc_dev_s *dev)
{
  return trap_stop_running(dev);
}

/****************************************************************************
 * Name: trap_params_set / trap_mode_set / trap_limits_set / state / fault
 ****************************************************************************/

static int trap_params_set(FAR struct bldc_dev_s *dev,
                           FAR const struct motor_params_s *params)
{
  (void)dev;
  (void)params;

  return OK;
}

static int trap_mode_set(FAR struct bldc_dev_s *dev, uint8_t mode)
{
  (void)dev;

  switch (mode)
    {
      case MOTOR_OPMODE_INIT:
      case MOTOR_OPMODE_SPEED:
        return OK;

      default:
        return -ENOTSUP;
    }
}

static int trap_limits_set(FAR struct bldc_dev_s *dev,
                           FAR const struct motor_limits_s *limits)
{
  (void)dev;
  (void)limits;
  return OK;
}

static int trap_state_get(FAR struct bldc_dev_s *dev,
                          FAR struct motor_state_s *state)
{
  FAR struct trap_priv_s *priv = trap_priv(dev);

  DEBUGASSERT(state != NULL);

  if (!priv->armed)
    {
      state->state = MOTOR_STATE_INIT;
    }
  else if (priv->running)
    {
      state->state = MOTOR_STATE_RUN;
    }
  else
    {
      state->state = MOTOR_STATE_IDLE;
    }

  state->fault = dev->lower.state.fault;
  return OK;
}

static int trap_fault_get(FAR struct bldc_dev_s *dev,
                          FAR uint8_t *fault)
{
  DEBUGASSERT(fault != NULL);
  *fault = dev->lower.state.fault;
  return OK;
}

static int trap_fault_clear(FAR struct bldc_dev_s *dev, uint8_t fault)
{
  int ret = OK;

  if (dev->board != NULL && dev->board->fault_clear != NULL)
    {
      ret = dev->board->fault_clear(dev);
    }

  if (ret == OK)
    {
      dev->lower.state.fault &= ~fault;
    }

  return ret;
}

/****************************************************************************
 * Name: trap_ioctl
 ****************************************************************************/

static int trap_ioctl(FAR struct bldc_dev_s *dev, int cmd,
                      unsigned long arg)
{
  switch (cmd)
    {
#ifdef CONFIG_MOTOR_BLDC_DEBUG
      case BLDCIOC_DEBUG_TRAP_RUN:
        {
          FAR const struct bldc_debug_trap_run_s *r =
            (FAR const struct bldc_debug_trap_run_s *)arg;

          if (r == NULL)
            {
              return -EINVAL;
            }

          return trap_start_running(dev, r->duty, r->freq_electrical_hz,
                                    r->direction);
        }

      case BLDCIOC_DEBUG_TRAP_STOP:
        {
          return trap_stop_running(dev);
        }

      case BLDCIOC_DEBUG_TRAP_SECTOR:
        {
          FAR const struct bldc_debug_trap_sector_s *s =
            (FAR const struct bldc_debug_trap_sector_s *)arg;
          FAR struct trap_priv_s *priv = trap_priv(dev);

          if (s == NULL)
            {
              return -EINVAL;
            }

          if (priv->running)
            {
              (void)trap_stop_running(dev);
            }

          return trap_set_sector(dev, s->sector, s->duty);
        }

      case BLDCIOC_DEBUG_TRAP_STATUS:
        {
          FAR struct bldc_debug_trap_status_s *st =
            (FAR struct bldc_debug_trap_status_s *)arg;

          if (st == NULL)
            {
              return -EINVAL;
            }

          trap_status_get(dev, st);
          return OK;
        }
#endif /* CONFIG_MOTOR_BLDC_DEBUG */

      case BLDCIOC_TRAP_SENSORLESS_CFG_SET:
        {
          FAR const struct bldc_trap_sensorless_cfg_s *c =
            (FAR const struct bldc_trap_sensorless_cfg_s *)arg;
          FAR struct trap_priv_s *priv = trap_priv(dev);
          irqstate_t flags;

          if (c == NULL || c->pole_pairs == 0)
            {
              return -EINVAL;
            }

          flags = enter_critical_section();
          priv->cfg = *c;
          priv->min_revs_per_tick =
              trap_compute_min_revs_per_tick(priv->pwm_freq,
                                             priv->cfg.min_rps_electrical);
          leave_critical_section(flags);
          return OK;
        }

      case BLDCIOC_TRAP_SENSORLESS_CFG_GET:
        {
          FAR struct bldc_trap_sensorless_cfg_s *c =
            (FAR struct bldc_trap_sensorless_cfg_s *)arg;
          FAR struct trap_priv_s *priv = trap_priv(dev);

          if (c == NULL)
            {
              return -EINVAL;
            }

          *c = priv->cfg;
          return OK;
        }

      case BLDCIOC_TRAP_SENSORLESS_DRIVE:
        {
          FAR const struct bldc_trap_sensorless_drive_s *d =
            (FAR const struct bldc_trap_sensorless_drive_s *)arg;
          FAR struct trap_priv_s *priv = trap_priv(dev);

          if (d == NULL)
            {
              return -EINVAL;
            }

          if (!priv->running ||
              priv->state == BLDC_TRAP_STATE_IDLE ||
              priv->state == BLDC_TRAP_STATE_FIXED)
            {
              return trap_start_closed_loop(dev, d->amplitude_q15);
            }

          return trap_update_drive(dev, d->amplitude_q15);
        }

      case BLDCIOC_TRAP_SENSORLESS_STATE_GET:
        {
          FAR struct bldc_trap_sensorless_state_s *st =
            (FAR struct bldc_trap_sensorless_state_s *)arg;
          FAR struct trap_priv_s *priv = trap_priv(dev);
          irqstate_t flags;
          float mech_rps;
          int16_t err_deg_i16;
          uint16_t var_deg2_u16;

          if (st == NULL)
            {
              return -EINVAL;
            }

          flags = enter_critical_section();

          if (priv->cfg.pole_pairs > 0)
            {
              mech_rps = priv->m_mech_revs_per_tick *
                         (float)priv->pwm_freq /
                         (float)priv->cfg.pole_pairs;
            }
          else
            {
              mech_rps = 0.0f;
            }

          err_deg_i16 = (int16_t)priv->phase_err_deg;
          if (priv->phase_err_deg > 32767.0f)
            {
              err_deg_i16 = 32767;
            }
          else if (priv->phase_err_deg < -32768.0f)
            {
              err_deg_i16 = -32768;
            }

          var_deg2_u16 = (priv->phase_err_var_deg2 > 65535.0f)
                         ? 65535
                         : (uint16_t)priv->phase_err_var_deg2;

          st->state              = priv->state;
          st->sector             = priv->sector;
          st->direction          = priv->direction;
          st->stalled            = priv->stalled;
          st->amplitude_q15      = priv->amp_q15;
          st->mech_rps_q16       = (int32_t)(mech_rps * 65536.0f);
          st->phase_error_deg    = err_deg_i16;
          st->phase_err_var_deg2 = var_deg2_u16;
          st->commutations       = priv->commutations;

          leave_critical_section(flags);
          return OK;
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct bldc_backend_ops_s g_bldc_trap_ops =
{
  .name             = "trap",
  .control_mode     = MOTOR_CONTROL_TRAP,
  .capabilities     = MOTOR_CAP_ALGO_TRAPEZOIDAL,
  .setup            = trap_setup,
  .shutdown         = trap_shutdown,
  .start            = trap_start,
  .stop             = trap_stop,
  .params_set       = trap_params_set,
  .limits_set       = trap_limits_set,
  .mode_set         = trap_mode_set,
  .state_get        = trap_state_get,
  .fault_get        = trap_fault_get,
  .fault_clear      = trap_fault_clear,
  .ioctl            = trap_ioctl,
  .commutation_tick = trap_commutation_tick,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR const struct bldc_backend_ops_s *bldc_trap_get_ops(void)
{
  return &g_bldc_trap_ops;
}
