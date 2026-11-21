/****************************************************************************
 * boards/arm/stm32u3/nucleo-u3c5zi-q/src/stm32_adc.c
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

#include <sys/param.h>

#include <errno.h>
#include <stdbool.h>

#include <arch/board/board.h>
#include <nuttx/analog/adc.h>
#include <nuttx/debug.h>

#include "stm32.h"
#include "nucleo-u3c5zi-q.h"

#if defined(CONFIG_ADC) && defined(CONFIG_STM32_ADC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC1_NCHANNELS 3

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* VREFINT (about 1.2 V, needs no wiring), PA0 and PA1 */

static const uint8_t g_chanlist1[ADC1_NCHANNELS] =
{
  ADC_CH_VREFINT,
  3,
  4
};

static const uint32_t g_pinlist1[] =
{
  GPIO_ADC1_IN3_1,  /* PA0 */
  GPIO_ADC1_IN4_1   /* PA1 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC1 and register it as /dev/adc0.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc1;
  int ret;
  int i;

  if (initialized)
    {
      return OK;
    }

  for (i = 0; i < nitems(g_pinlist1); i++)
    {
      stm32_configgpio(g_pinlist1[i]);
    }

  adc1 = stm32_adc_initialize(1, g_chanlist1, ADC1_NCHANNELS);
  if (adc1 == NULL)
    {
      aerr("ERROR: Failed to get ADC interface 1\n");
      return -ENODEV;
    }

  ret = adc_register("/dev/adc0", adc1);
  if (ret < 0)
    {
      aerr("ERROR: adc_register /dev/adc0 failed: %d\n", ret);
      return ret;
    }

  initialized = true;
  return OK;
}

#endif /* CONFIG_ADC && CONFIG_STM32_ADC1 */
