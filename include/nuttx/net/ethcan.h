/****************************************************************************
 * include/nuttx/net/ethcan.h
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

#ifndef __INCLUDE_NUTTX_NET_ETHCAN_H
#define __INCLUDE_NUTTX_NET_ETHCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_ETHCAN

#include <stdint.h>

#include <nuttx/can/can.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ethcan_initialize
 *
 * Description:
 *   Bind a CAN FD controller's lower-half interface to an ethcan network
 *   interface.  The caller (normally board bring-up logic) is responsible
 *   for obtaining 'candev' from that controller's own lower-half
 *   initialization function (e.g. an xxx_caninitialize()-style call) and
 *   for choosing 'lane' and 'macaddr'.
 *
 *   'candev' must NOT also be passed to can_register(): a controller
 *   bound to ethcan is dedicated to it and does not appear as /dev/canX.
 *
 * Input Parameters:
 *   intf    - If there are multiple ethcan interfaces, this value
 *             identifies which is being initialized. The number of
 *             supported interfaces is CONFIG_NET_ETHCAN_NINTERFACES.
 *   candev  - The lower-half CAN FD controller to bind to this interface.
 *   macaddr - The 6-byte Ethernet MAC address for this interface.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ethcan_initialize(int intf, FAR struct can_dev_s *candev,
                       FAR const uint8_t *macaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ETHCAN */
#endif /* __INCLUDE_NUTTX_NET_ETHCAN_H */
