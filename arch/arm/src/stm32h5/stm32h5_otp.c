/****************************************************************************
 * arch/arm/src/stm32h5/stm32h5_otp.c
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
 * The STM32H5 one-time-programmable area exposed through the NuttX efuse
 * interface.
 *
 * Per RM0481's "OTP area" section, the OTP is 2 KB at STM32_OTP_BASE
 * (0x08fff000, right below the unique-ID/flash-size block at
 * STM32_SYSMEM_UID), organized as 1024 x 16-bit words -- that 16-bit word
 * is the OTP's native unit, distinct from main flash's 32-bit word, and
 * this driver's "word" always refers to it.  Both 16- and 32-bit accesses
 * are supported by the hardware, but ECC is computed per 16-bit word, so
 * that is also the unit that can only ever be programmed once: because
 * OTP has no erase, a word that already holds any programmed bit cannot
 * be revisited without invalidating its own ECC.  This driver always
 * programs a whole word at a time and refuses to touch one that isn't
 * still blank, rather than attempt a partial reprogram.
 *
 * Reading has none of those constraints: it is a bare load, with no side
 * effects and no ordering requirement against writes elsewhere, so
 * stm32h5_otp_read() has no dependency on driver init order and is safe
 * to call from the earliest boot code.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arch/barriers.h>
#include <nuttx/efuse/efuse.h>

#include "arm_internal.h"
#include "stm32_flash.h"
#include "hardware/stm32_flash.h"
#include "hardware/stm32_memorymap.h"
#include "stm32h5_otp.h"

#ifdef CONFIG_STM32H5_OTP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OTP_NWORDS        STM32H5_OTP_NWORDS
#define OTP_WORD_BITS     STM32H5_OTP_WORD_BITS
#define OTP_WORD_BYTES     (OTP_WORD_BITS / 8)
#define OTP_TOTAL_BITS    STM32H5_OTP_TOTAL_BITS

#define OTP_ERASEDVALUE   0xffffu

#define OTP_TIMEOUT_VALUE 5000000                    /* 5s, in microsecond
                                                        * polls, matches
                                                        * FLASH_TIMEOUT_VALUE
                                                        * in
                                                        * stm32h563xx_flash.c
                                                        */

#if (OTP_NWORDS * OTP_WORD_BYTES) != STM32_OTP_SIZE
#  error "STM32H5_OTP_NWORDS/WORD_BITS disagree with STM32_OTP_SIZE"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32h5_otp_read_field(FAR struct efuse_lowerhalf_s *lower,
                                  FAR const efuse_desc_t *field[],
                                  FAR uint8_t *data, size_t bit_size);
static int stm32h5_otp_write_field(FAR struct efuse_lowerhalf_s *lower,
                                   FAR const efuse_desc_t *field[],
                                   FAR const uint8_t *data, size_t bit_size);
static int stm32h5_otp_ioctl(FAR struct efuse_lowerhalf_s *lower, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct efuse_ops_s g_stm32h5_otp_ops =
{
  .read_field  = stm32h5_otp_read_field,
  .write_field = stm32h5_otp_write_field,
  .ioctl       = stm32h5_otp_ioctl,
};

static struct efuse_lowerhalf_s g_stm32h5_otp_lower =
{
  .ops = &g_stm32h5_otp_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h5_otp_field_bits
 *
 * Description:
 *   Total number of bits described by a NULL terminated field list.
 *
 ****************************************************************************/

static size_t stm32h5_otp_field_bits(FAR const efuse_desc_t *field[])
{
  size_t bits = 0;
  int i;

  for (i = 0; field[i] != NULL; i++)
    {
      bits += field[i]->bit_count;
    }

  return bits;
}

/****************************************************************************
 * Name: stm32h5_otp_check_field
 *
 * Description:
 *   Verify that every descriptor lies inside the OTP bit address space.
 *
 ****************************************************************************/

static int stm32h5_otp_check_field(FAR const efuse_desc_t *field[])
{
  int i;

  for (i = 0; field[i] != NULL; i++)
    {
      if ((size_t)field[i]->bit_offset + field[i]->bit_count >
          OTP_TOTAL_BITS)
        {
          ferr("ERROR: field %d [%u,+%u) is outside the OTP\n", i,
               field[i]->bit_offset, field[i]->bit_count);
          return -EINVAL;
        }
    }

  return OK;
}

#ifdef CONFIG_STM32H5_OTP_WRITE
/****************************************************************************
 * Name: stm32h5_otp_wait_ready
 *
 * Description:
 *   Wait for any flash controller operation in progress to finish.
 *
 ****************************************************************************/

static int stm32h5_otp_wait_ready(void)
{
  int i;

  UP_DSB();

  for (i = 0; i < OTP_TIMEOUT_VALUE; i++)
    {
      if (!(getreg32(STM32_FLASH_NSSR) &
          (FLASH_NSSR_BSY | FLASH_NSSR_WBNE | FLASH_NSSR_DBNE)))
        {
          return OK;
        }

      up_udelay(1);
    }

  return -EBUSY;
}

/****************************************************************************
 * Name: stm32h5_otp_program_word
 *
 * Description:
 *   Program one 16-bit OTP word -- the unit ECC is computed over, and so
 *   the unit that can only ever be programmed once.  The caller must
 *   already have confirmed the word is currently blank; this function
 *   does not check, since by the time it is called the bits to preserve
 *   and the bits to clear have already been merged into "value".
 *
 ****************************************************************************/

static int stm32h5_otp_program_word(uint32_t word, uint16_t value)
{
  FAR uint16_t *wp;
  uint32_t addr;
  int ret;

  addr = STM32_OTP_BASE + word * OTP_WORD_BYTES;

  ret = stm32h5_otp_wait_ready();
  if (ret < 0)
    {
      return ret;
    }

  stm32_flash_unlock();

  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_PG);

  wp = (FAR uint16_t *)addr;

  UP_MB();

  *wp = value;

  /* Data synchronous barrier just after the write, so the CPU cannot
   * reorder the status-register poll ahead of the store above.
   */

  UP_MB();

  ret = stm32h5_otp_wait_ready();
  if (ret == OK && (getreg32(STM32_FLASH_ECCDETR) & FLASH_ECCDETR_ECCD))
    {
      modifyreg32(STM32_FLASH_ECCDETR, 0, FLASH_ECCDETR_ECCD);
      ret = -EIO;
    }

  modifyreg32(STM32_FLASH_NSCR, FLASH_NSCR_PG, 0);
  modifyreg32(STM32_FLASH_NSCCR, 0, ~0);

  stm32_flash_lock();

  if (ret < 0)
    {
      return ret;
    }

  if (getreg16(addr) != value)
    {
      return -EIO;
    }

  return OK;
}
#endif /* CONFIG_STM32H5_OTP_WRITE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h5_otp_read
 ****************************************************************************/

int stm32h5_otp_read(uint32_t word, FAR uint16_t *value)
{
  if (value == NULL)
    {
      return -EINVAL;
    }

  if (word >= OTP_NWORDS)
    {
      return -EINVAL;
    }

  *value = getreg16(STM32_OTP_BASE + word * OTP_WORD_BYTES);
  return OK;
}

/****************************************************************************
 * Name: stm32h5_otp_read_field
 *
 * Description:
 *   Read the bits named by the field list.  The bits are packed towards
 *   the start of the caller's buffer: the first bit of the first
 *   descriptor lands in bit 0 of data[0], the next in bit 1, and so on
 *   across descriptor boundaries.
 *
 ****************************************************************************/

static int stm32h5_otp_read_field(FAR struct efuse_lowerhalf_s *lower,
                                  FAR const efuse_desc_t *field[],
                                  FAR uint8_t *data, size_t bit_size)
{
  uint32_t cached_word = UINT32_MAX;
  uint16_t cached_val = 0;
  size_t written = 0;
  size_t request;
  int ret;
  int i;

  if (field == NULL || data == NULL)
    {
      return -EINVAL;
    }

  ret = stm32h5_otp_check_field(field);
  if (ret < 0)
    {
      return ret;
    }

  request = stm32h5_otp_field_bits(field);
  if (bit_size != 0 && bit_size < request)
    {
      request = bit_size;
    }

  memset(data, 0, (request + 7) / 8);

  for (i = 0; field[i] != NULL && written < request; i++)
    {
      size_t bit;

      for (bit = 0; bit < field[i]->bit_count && written < request;
           bit++, written++)
        {
          size_t   flat = field[i]->bit_offset + bit;
          uint32_t word = flat / OTP_WORD_BITS;

          if (word != cached_word)
            {
              ret = stm32h5_otp_read(word, &cached_val);
              if (ret < 0)
                {
                  return ret;
                }

              cached_word = word;
            }

          if ((cached_val & (1u << (flat % OTP_WORD_BITS))) != 0)
            {
              data[written / 8] |= 1u << (written % 8);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32h5_otp_write_field
 *
 * Description:
 *   Program the bits named by the field list, taking the data in the same
 *   packed layout that stm32h5_otp_read_field() produces.
 *
 *   This is destructive and irreversible.  Programming happens a whole
 *   16-bit word at a time because that is the unit ECC is computed over,
 *   which also means a word that already holds any programmed bit cannot
 *   be touched again: such a write is rejected here rather than left to
 *   corrupt its ECC.
 *
 ****************************************************************************/

static int stm32h5_otp_write_field(FAR struct efuse_lowerhalf_s *lower,
                                   FAR const efuse_desc_t *field[],
                                   FAR const uint8_t *data, size_t bit_size)
{
#ifndef CONFIG_STM32H5_OTP_WRITE
  /* Programming is not built in.  Refuse rather than silently doing
   * nothing, so a caller cannot mistake this for a successful burn.
   */

  return -EPERM;
#else
  uint32_t word = UINT32_MAX;
  uint16_t value = 0;
  bool dirty = false;
  size_t consumed = 0;
  size_t request;
  int ret;
  int i;

  if (field == NULL || data == NULL)
    {
      return -EINVAL;
    }

  ret = stm32h5_otp_check_field(field);
  if (ret < 0)
    {
      return ret;
    }

  request = stm32h5_otp_field_bits(field);
  if (bit_size != 0 && bit_size < request)
    {
      request = bit_size;
    }

  for (i = 0; field[i] != NULL && consumed < request; i++)
    {
      size_t bit;

      for (bit = 0; bit < field[i]->bit_count && consumed < request;
           bit++, consumed++)
        {
          size_t   flat     = field[i]->bit_offset + bit;
          uint32_t new_word = flat / OTP_WORD_BITS;
          size_t   wordbit  = flat % OTP_WORD_BITS;

          if (new_word != word)
            {
              if (dirty)
                {
                  ret = stm32h5_otp_program_word(word, value);
                  if (ret < 0)
                    {
                      return ret;
                    }
                }

              /* Load the word's current contents and insist it is
               * untouched before we start clearing bits in our local
               * copy -- a word can only ever be programmed once.
               */

              ret = stm32h5_otp_read(new_word, &value);
              if (ret < 0)
                {
                  return ret;
                }

              if (value != OTP_ERASEDVALUE)
                {
                  ferr("ERROR: OTP word %" PRIu32 " already programmed, "
                       "its ECC cannot be recomputed\n", new_word);
                  return -EROFS;
                }

              word = new_word;
              dirty = false;
            }

          if ((data[consumed / 8] & (1u << (consumed % 8))) == 0)
            {
              value &= ~(1u << wordbit);
              dirty = true;
            }
        }
    }

  if (dirty)
    {
      ret = stm32h5_otp_program_word(word, value);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
#endif /* CONFIG_STM32H5_OTP_WRITE */
}

/****************************************************************************
 * Name: stm32h5_otp_ioctl
 ****************************************************************************/

static int stm32h5_otp_ioctl(FAR struct efuse_lowerhalf_s *lower, int cmd,
                             unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: stm32h5_otp_initialize
 ****************************************************************************/

int stm32h5_otp_initialize(FAR const char *devpath)
{
  FAR void *handle;

  handle = efuse_register(devpath, &g_stm32h5_otp_lower);
  if (handle == NULL)
    {
      ferr("ERROR: failed to register the OTP at %s\n", devpath);
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_STM32H5_OTP */
