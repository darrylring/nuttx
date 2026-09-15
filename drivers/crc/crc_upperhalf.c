/****************************************************************************
 * drivers/crc/crc_upperhalf.c
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
#include <string.h>
#include <sys/param.h>

#include <nuttx/crc/crc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-open() session.  Each open file descriptor gets its own running CRC
 * value and profile, so multiple sessions can stream independently while
 * time-sharing the single physical engine.
 */

struct crc_session_s
{
  struct crc_config_s cfg;        /* This session's configured profile */
  uint32_t  state;                /* Raw (un-reflected) running value */
  bool      started;              /* True once at least one chunk has
                                    * been fed since the last (re)seed */
  bool      configured;           /* True once CRCIOC_CONFIG has run */
};

/* The registered device.  One instance backs all sessions opened against
 * a given "/dev/crcN" node.
 */

struct crc_upperhalf_s
{
  FAR struct crc_lowerhalf_s *lower;  /* Lower half binding */
  mutex_t                     lock;   /* Serializes hardware access and
                                        * open/close reference counting */
  uint8_t                     ocount; /* Number of open sessions */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     crc_open(FAR struct file *filep);
static int     crc_close(FAR struct file *filep);
static ssize_t crc_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t crc_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     crc_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_crc_fops =
{
  crc_open,   /* open */
  crc_close,  /* close */
  crc_read,   /* read */
  crc_write,  /* write */
  NULL,       /* seek */
  crc_ioctl,  /* ioctl */
  NULL,       /* mmap */
  NULL,       /* truncate */
  NULL        /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc_reflect
 *
 * Description:
 *   Bit-reverse the low 'width' bits of 'value'.  Used to apply a
 *   session's 'refout' in software -- the hardware's own output-reversal
 *   bit is never used, since it would corrupt a resumed multi-chunk
 *   stream (see the lower half for details).
 *
 ****************************************************************************/

static uint32_t crc_reflect(uint32_t value, uint8_t width)
{
  uint32_t result = 0;
  uint8_t i;

  for (i = 0; i < width; i++)
    {
      if ((value & (1ul << i)) != 0)
        {
          result |= 1ul << (width - 1 - i);
        }
    }

  return result;
}

/****************************************************************************
 * Name: crc_finalize
 *
 * Description:
 *   Compute the reported CRC result for a session's current running
 *   value, applying the configured 'refout' and 'xorout'.  Does not
 *   modify the session, so it is safe to call mid-stream.
 *
 ****************************************************************************/

static uint32_t crc_finalize(FAR struct crc_session_s *session)
{
  uint32_t value = session->state;

  if (session->cfg.refout)
    {
      value = crc_reflect(value, session->cfg.width);
    }

  return value ^ session->cfg.xorout;
}

/****************************************************************************
 * Name: crc_compute
 *
 * Description:
 *   Feed 'len' bytes at 'buffer' into the shared hardware engine on
 *   behalf of 'session', resuming its previously checkpointed state (or
 *   its configured 'init' if this is the first chunk of a stream).
 *
 *   The hardware mutex is held for the full duration of the underlying
 *   write() call, including any DMA completion wait the lower half may
 *   do internally, so no other session's checkpoint/feed can interleave
 *   mid-burst.
 *
 ****************************************************************************/

static int crc_compute(FAR struct crc_upperhalf_s *upper,
                       FAR struct crc_session_s *session,
                       FAR const uint8_t *buffer, size_t len)
{
  FAR struct crc_lowerhalf_s *lower = upper->lower;
  uint32_t state;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* The engine's polynomial/width/refin registers are shared, global
   * state -- reprogram them on every access in case another session
   * with a different profile ran in between.
   */

  ret = lower->cl_ops->configure(lower, &session->cfg);
  if (ret < 0)
    {
      goto errout;
    }

  state = session->started ? session->state : session->cfg.init;
  ret = lower->cl_ops->checkpoint_load(lower, state);
  if (ret < 0)
    {
      goto errout;
    }

  ret = lower->cl_ops->write(lower, buffer, len, &state);

  if (ret >= 0)
    {
      session->state   = state;
      session->started = true;
    }

errout:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static int crc_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct crc_upperhalf_s *upper = inode->i_private;
  FAR struct crc_session_s *session;
  int ret;

  session = kmm_zalloc(sizeof(*session));
  if (session == NULL)
    {
      return -ENOMEM;
    }

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      kmm_free(session);
      return ret;
    }

  if (upper->ocount == 0)
    {
      ret = upper->lower->cl_ops->setup(upper->lower);
      if (ret < 0)
        {
          nxmutex_unlock(&upper->lock);
          kmm_free(session);
          return ret;
        }
    }

  upper->ocount++;
  nxmutex_unlock(&upper->lock);

  filep->f_priv = session;
  return OK;
}

static int crc_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct crc_upperhalf_s *upper = inode->i_private;
  FAR struct crc_session_s *session = filep->f_priv;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (--upper->ocount == 0)
    {
      upper->lower->cl_ops->shutdown(upper->lower);
    }

  nxmutex_unlock(&upper->lock);

  filep->f_priv = NULL;
  kmm_free(session);
  return OK;
}

static ssize_t crc_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct crc_session_s *session = filep->f_priv;
  uint32_t result;

  if (!session->configured)
    {
      return -EINVAL;
    }

  result = crc_finalize(session);
  buflen = MIN(buflen, sizeof(result));
  memcpy(buffer, &result, buflen);
  return buflen;
}

static ssize_t crc_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct crc_upperhalf_s *upper = inode->i_private;
  FAR struct crc_session_s *session = filep->f_priv;
  int ret;

  if (!session->configured)
    {
      return -EINVAL;
    }

  if (buflen == 0)
    {
      return 0;
    }

  ret = crc_compute(upper, session, (FAR const uint8_t *)buffer, buflen);
  return ret < 0 ? ret : (ssize_t)buflen;
}

static int crc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct crc_upperhalf_s *upper = inode->i_private;
  FAR struct crc_session_s *session = filep->f_priv;
  int ret;

  switch (cmd)
    {
      case CRCIOC_CONFIG:
        {
          FAR const struct crc_config_s *config =
            (FAR const struct crc_config_s *)arg;

          if (config == NULL)
            {
              return -EINVAL;
            }

          ret = nxmutex_lock(&upper->lock);
          if (ret < 0)
            {
              return ret;
            }

          ret = upper->lower->cl_ops->configure(upper->lower, config);
          nxmutex_unlock(&upper->lock);
          if (ret >= 0)
            {
              session->cfg        = *config;
              session->state      = config->init;
              session->started    = false;
              session->configured = true;
            }
        }
        break;

      case CRCIOC_RESET:
        if (!session->configured)
          {
            return -EINVAL;
          }

        session->state   = session->cfg.init;
        session->started = false;
        ret = OK;
        break;

      case CRCIOC_RESULT:
        {
          FAR uint32_t *result = (FAR uint32_t *)arg;

          if (result == NULL || !session->configured)
            {
              return -EINVAL;
            }

          *result = crc_finalize(session);
          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int crc_register(FAR const char *path, FAR struct crc_lowerhalf_s *lower)
{
  FAR struct crc_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path != NULL && lower != NULL && lower->cl_ops != NULL);

  upper = kmm_zalloc(sizeof(*upper));
  if (upper == NULL)
    {
      return -ENOMEM;
    }

  upper->lower = lower;
  nxmutex_init(&upper->lock);

  ret = register_driver(path, &g_crc_fops, 0666, upper);
  if (ret < 0)
    {
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
    }

  return ret;
}
