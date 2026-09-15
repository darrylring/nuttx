/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_crc.c
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

#include <nuttx/crc/crc.h>
#include <nuttx/debug.h>

#include "stm32_crc.h"

#if defined(CONFIG_CRC)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_crc_setup
 *
 * Description:
 *   Initialize the hardware CRC engine and register the CRC driver at
 *   "/dev/crc0".
 *
 ****************************************************************************/

int stm32_crc_setup(void)
{
  FAR struct crc_lowerhalf_s *lower;
  int ret;

  lower = stm32h5_crc_initialize();

  ret = crc_register("/dev/crc0", lower);
  if (ret < 0)
    {
      _err("ERROR: crc_register /dev/crc0 failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_CRC */
