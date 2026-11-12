/****************************************************************************
 * arch/arm/src/stm32h5/stm32_crypto.c
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

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <crypto/cryptodev.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/stm32_crc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CRC32_POLY      0x04c11db7
#define CRC32_XOR_VALUE 0xffffffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_crc_session_s
{
  bool     inuse;
  uint32_t state;   /* CRC_DR value carried between updates */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_stm32_crc_lock = NXMUTEX_INITIALIZER;
static FAR struct stm32_crc_session_s *g_stm32_crc_sessions;
static uint32_t g_stm32_crc_nsessions;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_crc_reverse32
 *
 * Description:
 *   Reverse the bit order of a 32-bit value.
 *
 ****************************************************************************/

static uint32_t stm32_crc_reverse32(uint32_t val)
{
  val = ((val >> 1) & 0x55555555) | ((val & 0x55555555) << 1);
  val = ((val >> 2) & 0x33333333) | ((val & 0x33333333) << 2);
  val = ((val >> 4) & 0x0f0f0f0f) | ((val & 0x0f0f0f0f) << 4);
  val = ((val >> 8) & 0x00ff00ff) | ((val & 0x00ff00ff) << 8);
  return (val >> 16) | (val << 16);
}

/****************************************************************************
 * Name: stm32_crc_update
 *
 * Description:
 *   Feed a buffer into the CRC unit, starting from the session's state.
 *   The unit reflects each input byte (REV_IN), but not the output, so
 *   CRC_DR can be written back to CRC_INIT for the next update.
 *
 ****************************************************************************/

static void stm32_crc_update(FAR struct stm32_crc_session_s *session,
                             FAR const uint8_t *buf, size_t len)
{
  putreg32(CRC_CR_POLYSIZE_32 | CRC_CR_REVIN_BYTE, STM32_CRC_CR);
  putreg32(CRC32_POLY, STM32_CRC_POL);
  putreg32(session->state, STM32_CRC_INIT);
  modifyreg32(STM32_CRC_CR, 0, CRC_CR_RESET);

  while (len >= sizeof(uint32_t))
    {
      putreg32(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
               ((uint32_t)buf[2] << 8) | (uint32_t)buf[3], STM32_CRC_DR);
      buf += sizeof(uint32_t);
      len -= sizeof(uint32_t);
    }

  if (len >= 2)
    {
      putreg16(((uint16_t)buf[0] << 8) | buf[1], STM32_CRC_DR);
      buf += 2;
      len -= 2;
    }

  if (len > 0)
    {
      putreg8(buf[0], STM32_CRC_DR);
    }

  session->state = getreg32(STM32_CRC_DR);
}

/****************************************************************************
 * Name: stm32_newsession
 *
 * Description:
 *   Create a CRC-32 session.  An optional 32-bit key is the start value,
 *   as in the software CRC-32 (a previous result, 0 for a new CRC).
 *
 ****************************************************************************/

static int stm32_newsession(FAR uint32_t *sid, FAR struct cryptoini *cri)
{
  FAR struct stm32_crc_session_s *sessions;
  uint32_t startval = 0;
  uint32_t i;
  int ret;

  if (sid == NULL || cri == NULL || cri->cri_alg != CRYPTO_CRC32 ||
      cri->cri_next != NULL)
    {
      return -EINVAL;
    }

  if (cri->cri_klen == 32)
    {
      memcpy(&startval, cri->cri_key, sizeof(startval));
    }
  else if (cri->cri_klen != 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_stm32_crc_lock);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < g_stm32_crc_nsessions; i++)
    {
      if (!g_stm32_crc_sessions[i].inuse)
        {
          break;
        }
    }

  if (i == g_stm32_crc_nsessions)
    {
      uint32_t n = i == 0 ? 4 : i * 2;

      sessions = kmm_realloc(g_stm32_crc_sessions, n * sizeof(*sessions));
      if (sessions == NULL)
        {
          nxmutex_unlock(&g_stm32_crc_lock);
          return -ENOBUFS;
        }

      memset(&sessions[i], 0, (n - i) * sizeof(*sessions));
      g_stm32_crc_sessions  = sessions;
      g_stm32_crc_nsessions = n;
    }

  g_stm32_crc_sessions[i].inuse = true;
  g_stm32_crc_sessions[i].state =
    stm32_crc_reverse32(startval ^ CRC32_XOR_VALUE);
  *sid = i;

  nxmutex_unlock(&g_stm32_crc_lock);
  return OK;
}

/****************************************************************************
 * Name: stm32_freesession
 ****************************************************************************/

static int stm32_freesession(uint64_t tid)
{
  uint32_t sid = (uint32_t)tid;
  int ret;

  ret = nxmutex_lock(&g_stm32_crc_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (sid < g_stm32_crc_nsessions && g_stm32_crc_sessions[sid].inuse)
    {
      g_stm32_crc_sessions[sid].inuse = false;
    }
  else
    {
      ret = -EINVAL;
    }

  nxmutex_unlock(&g_stm32_crc_lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_process
 ****************************************************************************/

static int stm32_process(FAR struct cryptop *crp)
{
  FAR struct cryptodesc *crd = crp->crp_desc;
  FAR struct stm32_crc_session_s *session;
  uint32_t sid = (uint32_t)crp->crp_sid;
  uint32_t result;
  int ret;

  if (crd == NULL || crd->crd_alg != CRYPTO_CRC32 || crd->crd_next != NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_stm32_crc_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (sid >= g_stm32_crc_nsessions || !g_stm32_crc_sessions[sid].inuse)
    {
      ret = -ENOENT;
      goto out;
    }

  session = &g_stm32_crc_sessions[sid];

  if ((crd->crd_flags & CRD_F_UPDATE) != 0)
    {
      stm32_crc_update(session,
                       (FAR const uint8_t *)crp->crp_buf + crd->crd_skip,
                       crd->crd_len);
    }
  else if (crp->crp_mac != NULL)
    {
      result = stm32_crc_reverse32(session->state) ^ CRC32_XOR_VALUE;
      memcpy(crp->crp_mac, &result, sizeof(result));
    }
  else
    {
      ret = -EINVAL;
    }

out:
  nxmutex_unlock(&g_stm32_crc_lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hwcr_init
 *
 * Description:
 *   Register the CRC unit with the crypto framework as CRYPTO_CRC32.
 *
 ****************************************************************************/

void hwcr_init(void)
{
  int algs[CRYPTO_ALGORITHM_MAX + 1];
  int hwcr_id;

  hwcr_id = crypto_get_driverid(0);
  DEBUGASSERT(hwcr_id >= 0);

  memset(algs, 0, sizeof(algs));
  algs[CRYPTO_CRC32] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(hwcr_id, algs, stm32_newsession,
                  stm32_freesession, stm32_process);
}
