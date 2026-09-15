/****************************************************************************
 * include/nuttx/crc/crc.h
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

#ifndef __INCLUDE_NUTTX_CRC_CRC_H
#define __INCLUDE_NUTTX_CRC_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRC driver ioctl commands ************************************************/

/* CRCIOC_CONFIG - Program the CRC profile (width, polynomial, seed,
 *                  reflection, final XOR) used by this open file's
 *                  session.  May be issued more than once on the same
 *                  file descriptor to switch profiles between streams.
 *
 *                  arg: A pointer to a populated struct crc_config_s.
 *
 * CRCIOC_RESET   - Re-seed the session's running CRC value back to the
 *                  configured 'init', without changing the configured
 *                  profile.  Used to start a new stream on an
 *                  already-open, already-configured file descriptor.
 *
 *                  arg: None.
 *
 * CRCIOC_RESULT  - Read back the finalized CRC result for the data fed
 *                  so far (with the configured 'refout'/'xorout' applied
 *                  in software).  Does not disturb the session's running
 *                  state, so more data may be written afterwards.
 *
 *                  arg: A pointer to a uint32_t to receive the result.
 */

#define CRCIOC_CONFIG  _CRCIOC(0x0001)
#define CRCIOC_RESET   _CRCIOC(0x0002)
#define CRCIOC_RESULT  _CRCIOC(0x0003)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the CRC algorithm to use for one session
 * (i.e. one open file descriptor).  It is copied into the driver's own
 * per-session state by CRCIOC_CONFIG; callers never hand the driver an
 * opaque context of their own.
 */

struct crc_config_s
{
  uint8_t  width;   /* Polynomial width in bits: 32, 16, 8, or 7 */
  uint32_t poly;    /* Polynomial value, MSB-first, implicit leading 1 */
  uint32_t init;    /* Seed value loaded into the engine before the first
                      * chunk of a stream */
  bool     refin;   /* Reflect each input byte before it is fed in */
  bool     refout;  /* Reflect the final accumulated value before it is
                      * reported as the result (applied in software only
                      * -- never via a hardware "reverse output" bit,
                      * since that would corrupt any resumed multi-chunk
                      * stream) */
  uint32_t xorout;  /* Value XORed with the (possibly reflected) result
                      * before it is reported */
};

struct crc_lowerhalf_s;

/* This structure defines all of the operations provided by the
 * architecture-specific "lower half" logic.  All fields must be provided
 * with non-NULL function pointers.  Whether and when 'write' uses DMA
 * internally is entirely up to the lower half -- the upper half has no
 * notion of DMA at all.
 */

struct crc_ops_s
{
  /* Enable the hardware engine's clock and bring it to a known state.
   * Called once, when the first session on this device is opened.
   */

  CODE int (*setup)(FAR struct crc_lowerhalf_s *lower);

  /* Disable the hardware engine.  Called once, when the last open
   * session on this device is closed.
   */

  CODE void (*shutdown)(FAR struct crc_lowerhalf_s *lower);

  /* Program the engine for the given CRC profile (polynomial, width,
   * input reflection).  Does not touch the running/seed state.
   */

  CODE int (*configure)(FAR struct crc_lowerhalf_s *lower,
                        FAR const struct crc_config_s *config);

  /* Load 'state' as the engine's current running value, so that the
   * next write() call resumes a previously checkpointed stream (or
   * starts a fresh one, if 'state' is the session's configured
   * 'init').
   */

  CODE int (*checkpoint_load)(FAR struct crc_lowerhalf_s *lower,
                              uint32_t state);

  /* Feed 'len' bytes at 'buffer' into the engine, and return the
   * engine's raw running value (with no output reflection or XOR
   * applied) in '*result'.
   */

  CODE int (*write)(FAR struct crc_lowerhalf_s *lower,
                    FAR const uint8_t *buffer, size_t len,
                    FAR uint32_t *result);
};

/* This is the device structure used by the lower half.  The caller of
 * crc_register() must allocate and initialize this structure, setting
 * 'cl_ops' and 'cl_priv' before registration; the upper half neither
 * reads nor writes 'cl_priv'.
 */

struct crc_lowerhalf_s
{
  FAR const struct crc_ops_s *cl_ops;   /* Arch-specific operations */
  FAR void                   *cl_priv;  /* Used by the arch-specific logic */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: crc_register
 *
 * Description:
 *   Register a CRC driver.  This function binds an instance of a "lower
 *   half" CRC driver with the "upper half" CRC device and registers that
 *   device so that it can be used by application code as, e.g.,
 *   "/dev/crc0".
 *
 * Input Parameters:
 *   path  - The full path to register in the NuttX pseudo-filesystem.
 *           The recommended convention is "/dev/crc0", "/dev/crc1", etc.
 *   lower - A pointer to an instance of the lower half CRC driver.  This
 *           instance is bound to the upper half driver and must persist
 *           as long as the upper half driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int crc_register(FAR const char *path, FAR struct crc_lowerhalf_s *lower);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_CRC_CRC_H */
