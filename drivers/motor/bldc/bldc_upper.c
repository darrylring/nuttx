/****************************************************************************
 * drivers/motor/bldc/bldc_upper.c
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

/* Generic BLDC upper-half adapter between motor.h and a pluggable backend. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>

#include <nuttx/motor/bldc/bldc.h>
#include <nuttx/motor/motor.h>
#include <nuttx/motor/motor_ioctl.h>

#ifdef CONFIG_MOTOR_BLDC_DEBUG
#  include <nuttx/motor/bldc/bldc_debug.h>
#endif

#include <assert.h>
#include <errno.h>
#include <string.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bldc_setup(FAR struct motor_lowerhalf_s *lower);
static int bldc_shutdown(FAR struct motor_lowerhalf_s *lower);
static int bldc_stop(FAR struct motor_lowerhalf_s *lower);
static int bldc_start(FAR struct motor_lowerhalf_s *lower);
static int bldc_params_set(FAR struct motor_lowerhalf_s *lower,
                           FAR struct motor_params_s *params);
static int bldc_mode_set(FAR struct motor_lowerhalf_s *lower,
                         uint8_t mode);
static int bldc_limits_set(FAR struct motor_lowerhalf_s *lower,
                           FAR struct motor_limits_s *limits);
static int bldc_fault_set(FAR struct motor_lowerhalf_s *lower,
                          uint8_t fault);
static int bldc_state_get(FAR struct motor_lowerhalf_s *lower,
                          FAR struct motor_state_s *state);
static int bldc_fault_get(FAR struct motor_lowerhalf_s *lower,
                          FAR uint8_t *fault);
static int bldc_fault_clear(FAR struct motor_lowerhalf_s *lower,
                            uint8_t fault);
static int bldc_ioctl(FAR struct motor_lowerhalf_s *lower, int cmd,
                      unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct motor_ops_s g_bldc_motor_ops =
{
  .setup       = bldc_setup,
  .shutdown    = bldc_shutdown,
  .stop        = bldc_stop,
  .start       = bldc_start,
  .params_set  = bldc_params_set,
  .mode_set    = bldc_mode_set,
  .limits_set  = bldc_limits_set,
  .fault_set   = bldc_fault_set,
  .state_get   = bldc_state_get,
  .fault_get   = bldc_fault_get,
  .fault_clear = bldc_fault_clear,
  .ioctl       = bldc_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bldc_from_lower
 ****************************************************************************/

static inline FAR struct bldc_dev_s *
bldc_from_lower(FAR struct motor_lowerhalf_s *lower)
{
  return (FAR struct bldc_dev_s *)lower;
}

/****************************************************************************
 * Name: bldc_setup
 ****************************************************************************/

static int bldc_setup(FAR struct motor_lowerhalf_s *lower)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);
  int                    ret = OK;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->board != NULL);
  DEBUGASSERT(dev->backend != NULL);

  if (dev->board->setup != NULL)
    {
      ret = dev->board->setup(dev);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (dev->backend->setup != NULL)
    {
      ret = dev->backend->setup(dev);
      if (ret < 0)
        {
          if (dev->board->shutdown != NULL)
            {
              dev->board->shutdown(dev);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bldc_shutdown
 ****************************************************************************/

static int bldc_shutdown(FAR struct motor_lowerhalf_s *lower)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);

  DEBUGASSERT(dev != NULL);

  if (dev->backend != NULL && dev->backend->shutdown != NULL)
    {
      dev->backend->shutdown(dev);
    }

  if (dev->board != NULL && dev->board->shutdown != NULL)
    {
      dev->board->shutdown(dev);
    }

  dev->running = false;
  return OK;
}

/****************************************************************************
 * Name: bldc_start
 ****************************************************************************/

static int bldc_start(FAR struct motor_lowerhalf_s *lower)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);
  int                    ret = OK;

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

  if (dev->backend->start != NULL)
    {
      ret = dev->backend->start(dev);
    }

  if (ret == OK)
    {
      dev->running = true;
    }

  return ret;
}

/****************************************************************************
 * Name: bldc_stop
 ****************************************************************************/

static int bldc_stop(FAR struct motor_lowerhalf_s *lower)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);
  int                    ret = OK;

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

  if (dev->backend->stop != NULL)
    {
      ret = dev->backend->stop(dev);
    }

  dev->running = false;
  return ret;
}

/****************************************************************************
 * Name: bldc_params_set
 ****************************************************************************/

static int bldc_params_set(FAR struct motor_lowerhalf_s *lower,
                           FAR struct motor_params_s *params)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

  if (dev->backend->params_set != NULL)
    {
      return dev->backend->params_set(dev, params);
    }

  return OK;
}

/****************************************************************************
 * Name: bldc_mode_set
 ****************************************************************************/

static int bldc_mode_set(FAR struct motor_lowerhalf_s *lower, uint8_t mode)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

  if (dev->backend->mode_set != NULL)
    {
      return dev->backend->mode_set(dev, mode);
    }

  return OK;
}

/****************************************************************************
 * Name: bldc_limits_set
 ****************************************************************************/

static int bldc_limits_set(FAR struct motor_lowerhalf_s *lower,
                           FAR struct motor_limits_s *limits)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

  if (dev->backend->limits_set != NULL)
    {
      return dev->backend->limits_set(dev, limits);
    }

  return OK;
}

/****************************************************************************
 * Name: bldc_fault_set
 ****************************************************************************/

static int bldc_fault_set(FAR struct motor_lowerhalf_s *lower, uint8_t fault)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

  if (dev->backend->fault_set != NULL)
    {
      return dev->backend->fault_set(dev, fault);
    }

  lower->state.fault |= fault;
  return OK;
}

/****************************************************************************
 * Name: bldc_state_get
 ****************************************************************************/

static int bldc_state_get(FAR struct motor_lowerhalf_s *lower,
                          FAR struct motor_state_s *state)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);
  int                    ret = OK;

  DEBUGASSERT(dev != NULL && dev->backend != NULL && state != NULL);

  memset(state, 0, sizeof(*state));
  if (dev->backend->state_get != NULL)
    {
      ret = dev->backend->state_get(dev, state);
    }
  else
    {
      state->state = lower->state.state;
      state->fault = lower->state.fault;
      state->fb    = lower->state.fb;
    }

  state->control_mode = dev->backend->control_mode;
  state->capabilities = dev->backend->capabilities | dev->capabilities;

  lower->state.state        = state->state;
  lower->state.fault        = state->fault;
  lower->state.control_mode = state->control_mode;
  lower->state.capabilities = state->capabilities;
  lower->state.fb           = state->fb;

  return ret;
}

/****************************************************************************
 * Name: bldc_fault_get
 ****************************************************************************/

static int bldc_fault_get(FAR struct motor_lowerhalf_s *lower,
                          FAR uint8_t *fault)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);

  DEBUGASSERT(dev != NULL && dev->backend != NULL && fault != NULL);

  if (dev->backend->fault_get != NULL)
    {
      return dev->backend->fault_get(dev, fault);
    }

  *fault = lower->state.fault;
  return OK;
}

/****************************************************************************
 * Name: bldc_fault_clear
 ****************************************************************************/

static int bldc_fault_clear(FAR struct motor_lowerhalf_s *lower,
                            uint8_t fault)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);
  int                    ret = OK;

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

  if (dev->backend->fault_clear != NULL)
    {
      ret = dev->backend->fault_clear(dev, fault);
    }
  else if (dev->board != NULL && dev->board->fault_clear != NULL)
    {
      ret = dev->board->fault_clear(dev);
    }

  lower->state.fault &= ~fault;
  return ret;
}

#ifdef CONFIG_MOTOR_BLDC_DEBUG
/****************************************************************************
 * Name: bldc_handle_debug_ioctl
 ****************************************************************************/

static int bldc_handle_debug_ioctl(FAR struct bldc_dev_s *dev, int cmd,
                                   unsigned long arg)
{
  switch (cmd)
    {
      case BLDCIOC_DEBUG_PWM_DUTIES:
        {
          FAR const struct bldc_debug_pwm_duties_s *d =
            (FAR const struct bldc_debug_pwm_duties_s *)arg;

          if (d == NULL || dev->board->pwm_set_duties == NULL)
            {
              return -EINVAL;
            }

          return dev->board->pwm_set_duties(dev, d->duty_a, d->duty_b,
                                            d->duty_c);
        }

      case BLDCIOC_DEBUG_PWM_OUTPUT:
        {
          FAR const struct bldc_debug_pwm_output_s *o =
            (FAR const struct bldc_debug_pwm_output_s *)arg;

          if (o == NULL || dev->board->pwm_enable == NULL)
            {
              return -EINVAL;
            }

          return dev->board->pwm_enable(dev, o->enable);
        }

      case BLDCIOC_DEBUG_PWM_INFO:
        {
          FAR struct bldc_pwm_info_s *info =
            (FAR struct bldc_pwm_info_s *)arg;

          if (info == NULL || dev->board->pwm_get_info == NULL)
            {
              return -EINVAL;
            }

          return dev->board->pwm_get_info(dev, info);
        }

      case BLDCIOC_DEBUG_PWM_REGDUMP:
        {
          FAR struct bldc_debug_pwm_regdump_s *r =
            (FAR struct bldc_debug_pwm_regdump_s *)arg;

          if (r == NULL || dev->board->pwm_get_regdump == NULL)
            {
              return -EINVAL;
            }

          return dev->board->pwm_get_regdump(dev, r);
        }

      case BLDCIOC_DEBUG_DRV_DUMP:
        {
          FAR struct bldc_debug_drv_dump_s *d =
            (FAR struct bldc_debug_drv_dump_s *)arg;

          if (d == NULL || dev->board->drv_get_regdump == NULL)
            {
              return -EINVAL;
            }

          return dev->board->drv_get_regdump(dev, d);
        }

      case BLDCIOC_DEBUG_ADC_DUMP:
        {
          FAR struct bldc_debug_adc_dump_s *d =
            (FAR struct bldc_debug_adc_dump_s *)arg;

          if (d == NULL || dev->board->adc_get_snapshot == NULL)
            {
              return -EINVAL;
            }

          return dev->board->adc_get_snapshot(dev, d);
        }

      case BLDCIOC_DEBUG_BEMF_STATS:
        {
          FAR struct bldc_debug_bemf_stats_s *s =
            (FAR struct bldc_debug_bemf_stats_s *)arg;

          if (s == NULL || dev->board->bemf_get_stats == NULL)
            {
              return -EINVAL;
            }

          return dev->board->bemf_get_stats(dev, s);
        }

      default:
        return -ENOTTY;
    }
}
#endif /* CONFIG_MOTOR_BLDC_DEBUG */

/****************************************************************************
 * Name: bldc_ioctl
 ****************************************************************************/

static int bldc_ioctl(FAR struct motor_lowerhalf_s *lower, int cmd,
                      unsigned long arg)
{
  FAR struct bldc_dev_s *dev = bldc_from_lower(lower);
  int                    ret = -ENOTTY;

  DEBUGASSERT(dev != NULL && dev->backend != NULL);

#ifdef CONFIG_MOTOR_BLDC_DEBUG
  ret = bldc_handle_debug_ioctl(dev, cmd, arg);
  if (ret != -ENOTTY)
    {
      return ret;
    }
#endif

  if (dev->backend->ioctl != NULL)
    {
      ret = dev->backend->ioctl(dev, cmd, arg);
      if (ret != -ENOTTY)
        {
          return ret;
        }
    }

  if (dev->board != NULL && dev->board->ioctl != NULL)
    {
      ret = dev->board->ioctl(dev, cmd, arg);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bldc_register
 ****************************************************************************/

int bldc_register(FAR const char *path, FAR struct bldc_dev_s *dev)
{
  DEBUGASSERT(path != NULL);
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->board != NULL);
  DEBUGASSERT(dev->backend != NULL);

  dev->lower.ops = &g_bldc_motor_ops;

  dev->lower.state.state        = MOTOR_STATE_INIT;
  dev->lower.state.fault        = 0;
  dev->lower.state.control_mode = dev->backend->control_mode;
  dev->lower.state.capabilities = dev->backend->capabilities |
                                  dev->capabilities;

  dev->running = false;

  return motor_register(path, &dev->lower);
}
