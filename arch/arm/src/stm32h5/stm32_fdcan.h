/****************************************************************************
 * arch/arm/src/stm32h5/stm32_fdcan.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_FDCAN_H
#define __ARCH_ARM_SRC_STM32H5_STM32_FDCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_fdcan.h"

#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port numbers for use with stm32_fdcan_initialize() */

#define FDCAN1 1
#define FDCAN2 2

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(CONFIG_STM32_FDCAN_CHARDRIVER) || defined(CONFIG_STM32_FDCAN_ETHCAN)

/* Error and overflow counters kept by the driver.  Counts accumulate from
 * driver initialization until read with clear set.
 */

struct stm32_fdcan_counters_s
{
  uint32_t errors;       /* Protocol errors (ECR.CEL, accumulated) */
  uint32_t arb_errors;   /* Protocol errors in the arbitration phase */
  uint32_t data_errors;  /* Protocol errors in the data phase */
  uint32_t rx_overflows; /* Frames lost: RX FIFO or upper-half buffer full */
  uint32_t ep_entries;   /* Transitions into error passive */
  uint32_t bo_entries;   /* Transitions into bus-off */
  uint8_t  tec;          /* Current transmit error counter */
  uint8_t  rec;          /* Current receive error counter */
};

/* Receive hook, called from the FDCAN interrupt for every received frame
 * before it is passed to the upper-half driver.  Return true to consume the
 * frame, false to pass it on.
 */

typedef bool (*stm32_fdcan_rxhook_t)(FAR void *arg,
                                     FAR const struct can_hdr_s *hdr,
                                     FAR const uint8_t *data);

#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_STM32_FDCAN_CHARDRIVER

/****************************************************************************
 * Name: stm32_fdcaninitialize
 *
 * Description:
 *   Initialize the selected FDCAN port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple FDCAN interfaces)
 *
 * Returned Value:
 *   Valid FDCAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *stm32_fdcaninitialize(int port);

/****************************************************************************
 * Name: stm32_fdcan_setrxhook
 *
 * Description:
 *   Install (or, with hook == NULL, remove) a receive hook.  While a hook is
 *   installed, protocol error interrupts stay enabled so that every error
 *   is counted.
 *
 ****************************************************************************/

void stm32_fdcan_setrxhook(FAR struct can_dev_s *dev,
                           stm32_fdcan_rxhook_t hook, FAR void *arg);

/****************************************************************************
 * Name: stm32_fdcan_setbitrate
 *
 * Description:
 *   Set the nominal and data bit rates and sample points (in permille)
 *   without resetting the controller, so filters and pending state are
 *   kept.  Transmitter delay compensation is enabled for data rates above
 *   1 Mbit/s.
 *
 * Returned Value:
 *   Zero on success; -EINVAL if a rate cannot be made from the FDCAN
 *   kernel clock.
 *
 ****************************************************************************/

int stm32_fdcan_setbitrate(FAR struct can_dev_s *dev,
                           uint32_t nbitrate, uint16_t nsp,
                           uint32_t dbitrate, uint16_t dsp);

/****************************************************************************
 * Name: stm32_fdcan_checkbitrate
 *
 * Description:
 *   Check, without touching the hardware, that stm32_fdcan_setbitrate()
 *   would accept these rates and sample points.  If so, and they aren't
 *   NULL, nsp_actual and dsp_actual are set to the sample points (permille,
 *   rounded) it would really use: the nearest whole number of time quanta.
 *
 ****************************************************************************/

int stm32_fdcan_checkbitrate(uint32_t nbitrate, uint16_t nsp,
                             uint32_t dbitrate, uint16_t dsp,
                             FAR uint16_t *nsp_actual,
                             FAR uint16_t *dsp_actual);

/****************************************************************************
 * Name: stm32_fdcan_getbitrate
 *
 * Description:
 *   The bit rates and sample points (permille, rounded) the controller is
 *   set to: what stm32_fdcan_setbitrate() actually made of its request.
 *   Any output may be NULL.
 *
 ****************************************************************************/

void stm32_fdcan_getbitrate(FAR struct can_dev_s *dev,
                            FAR uint32_t *nbitrate, FAR uint16_t *nsp,
                            FAR uint32_t *dbitrate, FAR uint16_t *dsp);

/****************************************************************************
 * Name: stm32_fdcan_gettdc
 *
 * Description:
 *   Transmitter delay compensation, in FDCAN kernel clock periods (of
 *   clock_hz): the offset configured (0 while compensation is off, at data
 *   rates of 1 Mbit/s and below), and the last measured secondary sample
 *   point position.  The measured value is the loop delay through the
 *   transceiver plus the offset, updated by every frame sent with bit rate
 *   switching while compensation is on; it saturates at 127.
 *
 ****************************************************************************/

void stm32_fdcan_gettdc(FAR struct can_dev_s *dev, FAR uint8_t *tdco,
                        FAR uint8_t *tdcv, FAR uint32_t *clock_hz);

/****************************************************************************
 * Name: stm32_fdcan_setautoretransmit
 *
 * Description:
 *   Enable or disable automatic retransmission (CCCR.DAR).  With it
 *   disabled, a frame that loses arbitration or hits an error is dropped.
 *
 ****************************************************************************/

void stm32_fdcan_setautoretransmit(FAR struct can_dev_s *dev, bool enable);

/****************************************************************************
 * Name: stm32_fdcan_getcounters
 *
 * Description:
 *   Read the error counters, optionally clearing them.  TEC and REC are
 *   the controller's live values and are not cleared.
 *
 ****************************************************************************/

void stm32_fdcan_getcounters(FAR struct can_dev_s *dev,
                             FAR struct stm32_fdcan_counters_s *counters,
                             bool clear);
#endif

#ifdef CONFIG_STM32_FDCAN_SOCKET

/****************************************************************************
 * Name: stm32_fdcansockinitialize
 *
 * Description:
 *   Initialize the selected FDCAN port as SocketCAN interface
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple FDCAN interfaces)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int stm32_fdcansockinitialize(int port);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_FDCAN_H */
