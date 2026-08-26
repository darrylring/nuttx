/****************************************************************************
 * drivers/net/mdio.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/net/mdio.h>
#include <nuttx/debug.h>

/****************************************************************************
 * Private Defines
 ****************************************************************************/

#define MDIO_READ(d,a,r,v) d->lower->ops->read(d->lower, a, r, v);

#define MDIO_WRITE(d,a,r,v) d->lower->ops->write(d->lower, a, r, v);

#define MDIO_RESET(d,a) d->lower->ops->reset(d->lower, a);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the opaque handle used by application code to access the
 * MDIO bus.
 */

struct mdio_bus_s
{
  /* Pointer to the lower-half driver's state */

  FAR struct mdio_lowerhalf_s *lower;

  /* For exclusive access to the bus */

  mutex_t lock;

  /* Entry in g_mdio_buslist */

  struct list_node node;

  /* Number of mdio_register() callers sharing this bus */

  int refs;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single lower-half driver instance (i.e. a single MDIO controller) may
 * be shared by more than one caller, e.g. an Ethernet driver and board-
 * level bring-up code both talking to the same PHY bus.  g_mdio_buslist
 * tracks the buses that are currently registered so that a repeated
 * mdio_register() call on the same lower-half returns the existing,
 * already-locked bus instead of creating a second, unsynchronized one.
 * This mirrors the refcounted bus_initialize() pattern used by the I2C
 * and SPI upper halves.
 */

static struct list_node g_mdio_buslist = LIST_INITIAL_VALUE(g_mdio_buslist);
static mutex_t g_mdio_buslock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mdio_register
 *
 * Description:
 *   Register a new MDIO bus instance, or, if the given lower-half
 *   instance is already registered, return the existing bus instead.
 *   This allows independent callers (e.g. an Ethernet driver and board-
 *   level bring-up code) to safely share a single MDIO controller: every
 *   caller gets the same handle and the same lock, rather than each
 *   getting its own, unsynchronized view of the bus.
 *
 *   The bus remains registered until every caller that obtained a
 *   reference to it has released that reference with mdio_unregister().
 *
 * Input Parameters:
 *   lower - An instance of the lower-half MDIO driver, with the ops vtable
 *           as the first member.
 *
 * Returned Value:
 *   A non-NULL handle on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct mdio_bus_s *mdio_register(FAR struct mdio_lowerhalf_s *lower)
{
  FAR struct mdio_bus_s *dev;

  DEBUGASSERT(lower != NULL);

  nxmutex_lock(&g_mdio_buslock);

  /* Has this lower-half instance already been registered?  If so, share
   * it rather than creating a second, competing bus instance.
   */

  list_for_every_entry(&g_mdio_buslist, dev, struct mdio_bus_s, node)
    {
      if (dev->lower == lower)
        {
          dev->refs++;
          nxmutex_unlock(&g_mdio_buslock);
          return dev;
        }
    }

  /* Allocate the upper-half MDIO driver state structure */

  dev = (FAR struct mdio_bus_s *)kmm_zalloc(sizeof(struct mdio_bus_s));
  if (dev != NULL)
    {
      /* Initialize the upper-half driver state */

      nxmutex_init(&dev->lock);
      dev->lower = lower;
      dev->refs  = 1;
      list_add_tail(&g_mdio_buslist, &dev->node);
    }
  else
    {
      nerr("ERROR: Failed to allocate MDIO device structure\n");
    }

  nxmutex_unlock(&g_mdio_buslock);
  return dev;
}

/****************************************************************************
 * Name: mdio_unregister
 *
 * Description:
 *   Release a reference to an MDIO bus instance obtained from
 *   mdio_register().  The bus is only actually torn down once every
 *   caller that registered it has unregistered.
 *
 * Input Parameters:
 *   dev - The MDIO bus handle returned by mdio_register.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_unregister(FAR struct mdio_bus_s *dev)
{
  DEBUGASSERT(dev != NULL);

  nxmutex_lock(&g_mdio_buslock);

  DEBUGASSERT(dev->refs > 0);
  if (--dev->refs == 0)
    {
      list_delete(&dev->node);
      nxmutex_unlock(&g_mdio_buslock);

      nxmutex_destroy(&dev->lock);
      kmm_free(dev);
      return OK;
    }

  nxmutex_unlock(&g_mdio_buslock);
  return OK;
}

/****************************************************************************
 * Name: mdio_read
 *
 * Description:
 *   Read a 16-bit value from a PHY register on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - A pointer to the location to store the read value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr, uint8_t regaddr,
              FAR uint16_t *value)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->lower != NULL);
  DEBUGASSERT(dev->lower->ops->read != NULL);

  /* Take the mutex */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the lowerhalf driver's read method */

  ret = MDIO_READ(dev, phyaddr, regaddr, value);

  /* Release the mutex */

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mdio_write
 *
 * Description:
 *   Write a 16-bit value to a PHY register on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - The value to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_write(FAR struct mdio_bus_s *dev, uint8_t phyaddr, uint8_t regaddr,
               uint16_t value)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->lower != NULL);
  DEBUGASSERT(dev->lower->ops->write != NULL);

  /* Take the mutex */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the lowerhalf driver's write method */

  ret = MDIO_WRITE(dev, phyaddr, regaddr, value);

  /* Release the mutex */

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mdio_reset
 *
 * Description:
 *   Reset a PHY on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31) to reset.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_reset(FAR struct mdio_bus_s *dev, uint8_t phyaddr)
{
  int ret = -ENOSYS;

  DEBUGASSERT(dev != NULL && dev->lower != NULL);

  /* Check if the reset method is provided by the lower-half */

  if (dev->lower->ops->reset)
    {
      /* Take the mutex */

      ret = nxmutex_lock(&dev->lock);
      if (ret < 0)
        {
          return ret;
        }

      /* Call the lowerhalf driver's reset method */

      ret = MDIO_RESET(dev, phyaddr);

      /* Release the mutex */

      nxmutex_unlock(&dev->lock);
    }

  return ret;
}
