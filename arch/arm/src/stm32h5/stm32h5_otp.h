/****************************************************************************
 * arch/arm/src/stm32h5/stm32h5_otp.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32H5_OTP_H
#define __ARCH_ARM_SRC_STM32H5_STM32H5_OTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_STM32H5_OTP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The OTP area is 2 KB, organized as 1024 x 16-bit words -- RM0481, "OTP
 * area".  This is fixed by the silicon, not configurable per board, so
 * unlike STM32_OTP_BASE (arch-level, in hardware/stm32h5xxx_memorymap.h)
 * these geometry constants live here rather than in Kconfig.
 */

#define STM32H5_OTP_NWORDS     1024
#define STM32H5_OTP_WORD_BITS  16
#define STM32H5_OTP_TOTAL_BITS (STM32H5_OTP_NWORDS * STM32H5_OTP_WORD_BITS)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32h5_otp_read
 *
 * Description:
 *   Read one 16-bit word out of the OTP area (the OTP's native unit --
 *   see STM32H5_OTP_WORD_BITS above).
 *
 *   This is a bare memory-mapped load: it takes no lock, touches no other
 *   flash register, and has no dependency on any driver having been
 *   initialized.  It is safe to call as early as the OTP address window
 *   itself is mapped, i.e. from the very first C code that runs after
 *   reset -- well before up_initialize() or board bring-up register the
 *   /dev/efuse character device with stm32h5_otp_initialize().
 *
 * Input Parameters:
 *   word  - Word index, 0 to STM32H5_OTP_NWORDS - 1
 *   value - Receives the word's current contents
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32h5_otp_read(uint32_t word, FAR uint16_t *value);

/****************************************************************************
 * Name: stm32h5_otp_initialize
 *
 * Description:
 *   Register the OTP area as an efuse character device.  Call this once
 *   from board bring-up, after the usual driver/GPIO initialization has
 *   run -- unlike stm32h5_otp_read(), this allocates upper-half driver
 *   state and creates an inode, so it is not meant to run at early boot.
 *
 * Input Parameters:
 *   devpath - The path to the device, e.g. "/dev/efuse"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32h5_otp_initialize(FAR const char *devpath);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32H5_OTP */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32H5_OTP_H */
