/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_can.c
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

#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#ifdef CONFIG_NET_ETHCAN
#  include <nuttx/net/ethcan.h>
#endif

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "stm32_fdcan.h"
#include "nucleo-h563zi.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_STM32_FDCAN1) && !defined(CONFIG_STM32_FDCAN2)
#  error "No CAN device is enabled. Please enable at least one CAN device"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int stm32_can_setup(uint8_t port)
{
  struct can_dev_s *can;
  int ret;

  /* Call stm32_fdcaninitialize() to get an instance of the CAN interface */

  can = stm32_fdcaninitialize(port);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }

  if (port == 1)
    {
      ret = can_register("/dev/can0", can);
    }
  else if (port == 2)
    {
      ret = can_register("/dev/can1", can);
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}

#endif /* CONFIG_CAN */

#ifdef CONFIG_NET_ETHCAN

/****************************************************************************
 * Name: stm32_ethcan_setup
 *
 * Description:
 *   Initialize an FDCAN port and bind it to the ethcan network driver
 *   (CONFIG_NET_ETHCAN) instead of registering a /dev/canX character
 *   device -- the CONFIG_STM32_FDCAN_ETHCAN counterpart to
 *   stm32_can_setup() above.  The port must not also be passed to
 *   can_register(); the two are mutually exclusive uses of the same
 *   lower-half struct can_dev_s.
 *
 * Input Parameters:
 *   port    - 1 for FDCAN1, 2 for FDCAN2.
 *   intf    - ethcan interface index to register this port as (0-based;
 *             see CONFIG_NET_ETHCAN_NINTERFACES).
 *   macaddr - 6-byte Ethernet MAC address for this interface. This must
 *             be unique on the CAN segment -- the placeholder here is
 *             NOT: a real board should derive the low bytes from
 *             something per-device, e.g. the STM32 96-bit unique device
 *             ID, rather than ship this fixed value on more than one
 *             unit.
 *
 ****************************************************************************/

int stm32_ethcan_setup(uint8_t port, int intf)
{
  static const uint8_t macaddr[6] =
  {
    0x02, 0xe0, 0xde, 0xad, 0xbe, 0xef
  };

  struct can_dev_s *can;

  can = stm32_fdcaninitialize(port);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }

  return ethcan_initialize(intf, can, macaddr);
}

#endif /* CONFIG_NET_ETHCAN */
