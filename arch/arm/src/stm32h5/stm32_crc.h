/****************************************************************************
 * arch/arm/src/stm32h5/stm32_crc.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_CRC_H
#define __ARCH_ARM_SRC_STM32H5_STM32_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/crc/crc.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: stm32h5_crc_initialize
 *
 * Description:
 *   Bind the STM32H5 hardware CRC engine to a crc_lowerhalf_s instance,
 *   suitable for passing to crc_register().  The returned instance is a
 *   singleton -- there is exactly one physical CRC engine.
 *
 * Returned Value:
 *   A pointer to the lower half instance.  This function always succeeds.
 *
 ****************************************************************************/

FAR struct crc_lowerhalf_s *stm32h5_crc_initialize(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_STM32H5_STM32_CRC_H */
