/****************************************************************************
 * drivers/motor/gate_driver/drv83xx_core.c
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

#include <nuttx/spi/spi.h>

#include <nuttx/motor/gate_driver/drv83xx_core.h>

#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv83xx_lock_bus
 ****************************************************************************/

static void drv83xx_lock_bus(FAR struct drv83xx_dev_s *dev)
{
  SPI_LOCK(dev->spi, 1);
  SPI_SETBITS(dev->spi, 16);
  SPI_SETMODE(dev->spi, SPIDEV_MODE1);
  SPI_SETFREQUENCY(dev->spi, dev->freq);
}

/****************************************************************************
 * Name: drv83xx_unlock_bus
 ****************************************************************************/

static void drv83xx_unlock_bus(FAR struct drv83xx_dev_s *dev)
{
  SPI_LOCK(dev->spi, 0);
}

/****************************************************************************
 * Name: drv83xx_compose_drvctl
 ****************************************************************************/

static uint16_t drv83xx_compose_drvctl(FAR const struct drv83xx_cfg_s *cfg)
{
  uint16_t v = 0;

  v |= DRV83XX_DRVCTL_PWMMODE(cfg->pwm_mode);
  if (cfg->dis_cpuv)    v |= DRV83XX_DRVCTL_DIS_CPUV;
  if (cfg->dis_gdf)     v |= DRV83XX_DRVCTL_DIS_GDF;
  if (cfg->otw_rep)     v |= DRV83XX_DRVCTL_OTW_REP;
  if (cfg->onepwm_com)  v |= DRV83XX_DRVCTL_1PWM_COM;
  if (cfg->onepwm_dir)  v |= DRV83XX_DRVCTL_1PWM_DIR;
  return v;
}

/****************************************************************************
 * Name: drv83xx_compose_gdhs
 ****************************************************************************/

static uint16_t drv83xx_compose_gdhs(FAR const struct drv83xx_cfg_s *cfg)
{
  uint16_t v = 0;

  v |= DRV83XX_GDHS_LOCK_UNLOCKED;
  v |= DRV83XX_GDHS_IDRIVEP_HS(cfg->idrivep_hs);
  v |= DRV83XX_GDHS_IDRIVEN_HS(cfg->idriven_hs);
  return v;
}

/****************************************************************************
 * Name: drv83xx_compose_gdls
 ****************************************************************************/

static uint16_t drv83xx_compose_gdls(FAR const struct drv83xx_cfg_s *cfg)
{
  uint16_t v = 0;

  if (cfg->cbc) v |= DRV83XX_GDLS_CBC;
  v |= DRV83XX_GDLS_TDRIVE(cfg->tdrive);
  v |= DRV83XX_GDLS_IDRIVEP_LS(cfg->idrivep_ls);
  v |= DRV83XX_GDLS_IDRIVEN_LS(cfg->idriven_ls);
  return v;
}

/****************************************************************************
 * Name: drv83xx_compose_ocp
 ****************************************************************************/

static uint16_t drv83xx_compose_ocp(FAR const struct drv83xx_cfg_s *cfg)
{
  uint16_t v = 0;

  if (cfg->tretry) v |= DRV83XX_OCP_TRETRY;
  v |= DRV83XX_OCP_DEADTIME(cfg->dead_time);
  v |= DRV83XX_OCP_OCPMODE(cfg->ocp_mode);
  v |= DRV83XX_OCP_OCPDEG(cfg->ocp_deg);
  v |= DRV83XX_OCP_VDSLVL(cfg->vds_lvl);
  return v;
}

/****************************************************************************
 * Name: drv83xx_compose_csa
 ****************************************************************************/

static uint16_t drv83xx_compose_csa(FAR const struct drv83xx_cfg_s *cfg)
{
  uint16_t v = 0;

  if (cfg->csa_fet)   v |= DRV83XX_CSACTL_CSA_FET;
  if (cfg->vref_div)  v |= DRV83XX_CSACTL_VREFDIV;
  if (cfg->ls_ref)    v |= DRV83XX_CSACTL_LSREF;
  v |= DRV83XX_CSACTL_GAIN(cfg->csa_gain);
  if (cfg->dis_sen)   v |= DRV83XX_CSACTL_DIS_SEN;
  v |= DRV83XX_CSACTL_SENLVL(cfg->sen_lvl);
  return v;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv83xx_init
 ****************************************************************************/

void drv83xx_init(FAR struct drv83xx_dev_s *dev,
                  FAR struct spi_dev_s *spi,
                  int devno, uint32_t freq,
                  enum drv83xx_chip_e chip)
{
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(spi != NULL);

  dev->spi   = spi;
  dev->devno = devno;
  dev->freq  = freq;
  dev->chip  = (uint8_t)chip;
}

/****************************************************************************
 * Name: drv83xx_read_reg
 *
 * Description:
 *   Read an 11-bit register from the DRV83xx.  The response is shifted
 *   out on SDO during the same 16-bit frame (datasheet 8.5.1.1) so no CS
 *   toggle is needed between command and data.
 *
 ****************************************************************************/

void drv83xx_read_reg(FAR struct drv83xx_dev_s *dev,
                      uint8_t addr, FAR uint16_t *data)
{
  uint16_t tx = DRV83XX_SPI_W_READ |
                (((uint16_t)addr << DRV83XX_SPI_ADDR_SHIFT) &
                 DRV83XX_SPI_ADDR_MASK);
  uint16_t rx;

  DEBUGASSERT(dev != NULL && data != NULL);

  drv83xx_lock_bus(dev);
  SPI_SELECT(dev->spi, SPIDEV_MOTOR(dev->devno), true);

  rx = (uint16_t)SPI_SEND(dev->spi, tx);

  SPI_SELECT(dev->spi, SPIDEV_MOTOR(dev->devno), false);
  drv83xx_unlock_bus(dev);

  *data = rx & DRV83XX_SPI_DATA_MASK;
}

/****************************************************************************
 * Name: drv83xx_write_reg
 ****************************************************************************/

void drv83xx_write_reg(FAR struct drv83xx_dev_s *dev,
                       uint8_t addr, uint16_t data)
{
  uint16_t tx = DRV83XX_SPI_W_WRITE |
                (((uint16_t)addr << DRV83XX_SPI_ADDR_SHIFT) &
                 DRV83XX_SPI_ADDR_MASK) |
                (data & DRV83XX_SPI_DATA_MASK);

  DEBUGASSERT(dev != NULL);

  drv83xx_lock_bus(dev);
  SPI_SELECT(dev->spi, SPIDEV_MOTOR(dev->devno), true);

  SPI_SEND(dev->spi, tx);

  SPI_SELECT(dev->spi, SPIDEV_MOTOR(dev->devno), false);
  drv83xx_unlock_bus(dev);
}

/****************************************************************************
 * Name: drv83xx_modify_reg
 ****************************************************************************/

void drv83xx_modify_reg(FAR struct drv83xx_dev_s *dev,
                        uint8_t addr, uint16_t clr, uint16_t set)
{
  uint16_t regval = 0;

  drv83xx_read_reg(dev, addr, &regval);
  regval &= ~(clr & DRV83XX_SPI_DATA_MASK);
  regval |= (set & DRV83XX_SPI_DATA_MASK);
  drv83xx_write_reg(dev, addr, regval);
}

/****************************************************************************
 * Name: drv83xx_apply_cfg
 ****************************************************************************/

int drv83xx_apply_cfg(FAR struct drv83xx_dev_s *dev,
                      FAR const struct drv83xx_cfg_s *cfg)
{
  uint16_t tmp = 0;

  DEBUGASSERT(dev != NULL && cfg != NULL);

  /* Allow callers to refresh the SPI frequency from cfg */

  if (cfg->freq != 0)
    {
      dev->freq = cfg->freq;
    }

  /* Drain power-up status registers (they're read-only and self-clear on
   * read once the underlying condition is gone).
   */

  drv83xx_read_reg(dev, DRV83XX_REG_FAULT_STATUS1, &tmp);
  drv83xx_read_reg(dev, DRV83XX_REG_VGS_STATUS2,   &tmp);

  /* Program control registers from the cfg.  Write DRIVER_CONTROL first
   * (with CLR_FLT to wipe any latched faults) and GATE_DRIVE_HS with the
   * LOCK field unlocked so subsequent writes go through.
   */

  drv83xx_write_reg(dev, DRV83XX_REG_DRIVER_CONTROL,
                    drv83xx_compose_drvctl(cfg) | DRV83XX_DRVCTL_CLR_FLT);

  drv83xx_write_reg(dev, DRV83XX_REG_GATE_DRIVE_HS,
                    drv83xx_compose_gdhs(cfg));

  drv83xx_write_reg(dev, DRV83XX_REG_GATE_DRIVE_LS,
                    drv83xx_compose_gdls(cfg));

  drv83xx_write_reg(dev, DRV83XX_REG_OCP_CONTROL,
                    drv83xx_compose_ocp(cfg));

  drv83xx_write_reg(dev, DRV83XX_REG_CSA_CONTROL,
                    drv83xx_compose_csa(cfg));

  return OK;
}

/****************************************************************************
 * Name: drv83xx_get_faults
 ****************************************************************************/

int drv83xx_get_faults(FAR struct drv83xx_dev_s *dev,
                       FAR struct drv83xx_faults_s *out)
{
  DEBUGASSERT(dev != NULL && out != NULL);

  drv83xx_read_reg(dev, DRV83XX_REG_FAULT_STATUS1, &out->fs1);
  drv83xx_read_reg(dev, DRV83XX_REG_VGS_STATUS2,   &out->fs2);
  return OK;
}

/****************************************************************************
 * Name: drv83xx_clear_faults
 ****************************************************************************/

int drv83xx_clear_faults(FAR struct drv83xx_dev_s *dev)
{
  /* CLR_FLT self-clears after the write completes. */

  drv83xx_modify_reg(dev, DRV83XX_REG_DRIVER_CONTROL,
                     0, DRV83XX_DRVCTL_CLR_FLT);
  return OK;
}

/****************************************************************************
 * Name: drv83xx_set_gain
 ****************************************************************************/

int drv83xx_set_gain(FAR struct drv83xx_dev_s *dev, int gain)
{
  uint16_t set;

  switch (gain)
    {
      case 5:
        set = DRV83XX_CSACTL_GAIN_5;
        break;

      case 10:
        set = DRV83XX_CSACTL_GAIN_10;
        break;

      case 20:
        set = DRV83XX_CSACTL_GAIN_20;
        break;

      case 40:
        set = DRV83XX_CSACTL_GAIN_40;
        break;

      default:
        return -EINVAL;
    }

  drv83xx_modify_reg(dev, DRV83XX_REG_CSA_CONTROL,
                     DRV83XX_CSACTL_GAIN_MASK, set);
  return OK;
}

/****************************************************************************
 * Name: drv83xx_get_gain
 ****************************************************************************/

int drv83xx_get_gain(FAR struct drv83xx_dev_s *dev, FAR int *gain)
{
  uint16_t csa = 0;
  int      ret = OK;

  DEBUGASSERT(gain != NULL);

  drv83xx_read_reg(dev, DRV83XX_REG_CSA_CONTROL, &csa);

  switch (csa & DRV83XX_CSACTL_GAIN_MASK)
    {
      case DRV83XX_CSACTL_GAIN_5:
        *gain = 5;
        break;

      case DRV83XX_CSACTL_GAIN_10:
        *gain = 10;
        break;

      case DRV83XX_CSACTL_GAIN_20:
        *gain = 20;
        break;

      case DRV83XX_CSACTL_GAIN_40:
        *gain = 40;
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: drv83xx_set_idrive
 ****************************************************************************/

int drv83xx_set_idrive(FAR struct drv83xx_dev_s *dev,
                       FAR const struct drv83xx_idrive_s *id)
{
  DEBUGASSERT(id != NULL);

  drv83xx_modify_reg(dev, DRV83XX_REG_GATE_DRIVE_HS,
                     DRV83XX_GDHS_IDRIVEP_HS_MASK |
                       DRV83XX_GDHS_IDRIVEN_HS_MASK,
                     DRV83XX_GDHS_IDRIVEP_HS(id->idrivep_hs) |
                       DRV83XX_GDHS_IDRIVEN_HS(id->idriven_hs));

  drv83xx_modify_reg(dev, DRV83XX_REG_GATE_DRIVE_LS,
                     DRV83XX_GDLS_IDRIVEP_LS_MASK |
                       DRV83XX_GDLS_IDRIVEN_LS_MASK,
                     DRV83XX_GDLS_IDRIVEP_LS(id->idrivep_ls) |
                       DRV83XX_GDLS_IDRIVEN_LS(id->idriven_ls));
  return OK;
}

/****************************************************************************
 * Name: drv83xx_get_idrive
 ****************************************************************************/

int drv83xx_get_idrive(FAR struct drv83xx_dev_s *dev,
                       FAR struct drv83xx_idrive_s *id)
{
  uint16_t hs = 0;
  uint16_t ls = 0;

  DEBUGASSERT(id != NULL);

  drv83xx_read_reg(dev, DRV83XX_REG_GATE_DRIVE_HS, &hs);
  drv83xx_read_reg(dev, DRV83XX_REG_GATE_DRIVE_LS, &ls);

  id->idrivep_hs = (hs & DRV83XX_GDHS_IDRIVEP_HS_MASK) >>
                   DRV83XX_GDHS_IDRIVEP_HS_SHIFT;
  id->idriven_hs = (hs & DRV83XX_GDHS_IDRIVEN_HS_MASK) >>
                   DRV83XX_GDHS_IDRIVEN_HS_SHIFT;
  id->idrivep_ls = (ls & DRV83XX_GDLS_IDRIVEP_LS_MASK) >>
                   DRV83XX_GDLS_IDRIVEP_LS_SHIFT;
  id->idriven_ls = (ls & DRV83XX_GDLS_IDRIVEN_LS_MASK) >>
                   DRV83XX_GDLS_IDRIVEN_LS_SHIFT;
  return OK;
}

/****************************************************************************
 * Name: drv83xx_set_vds_lvl
 ****************************************************************************/

int drv83xx_set_vds_lvl(FAR struct drv83xx_dev_s *dev, int lvl)
{
  if (lvl < 0 || lvl > 0xf)
    {
      return -EINVAL;
    }

  drv83xx_modify_reg(dev, DRV83XX_REG_OCP_CONTROL,
                     DRV83XX_OCP_VDSLVL_MASK,
                     DRV83XX_OCP_VDSLVL(lvl));
  return OK;
}

/****************************************************************************
 * Name: drv83xx_get_vds_lvl
 ****************************************************************************/

int drv83xx_get_vds_lvl(FAR struct drv83xx_dev_s *dev, FAR int *lvl)
{
  uint16_t ocp = 0;

  DEBUGASSERT(lvl != NULL);

  drv83xx_read_reg(dev, DRV83XX_REG_OCP_CONTROL, &ocp);
  *lvl = (int)((ocp & DRV83XX_OCP_VDSLVL_MASK) >>
               DRV83XX_OCP_VDSLVL_SHIFT);
  return OK;
}

/****************************************************************************
 * Name: drv83xx_set_ocp_mode
 ****************************************************************************/

int drv83xx_set_ocp_mode(FAR struct drv83xx_dev_s *dev, int mode)
{
  if (mode < 0 || mode > 0x3)
    {
      return -EINVAL;
    }

  drv83xx_modify_reg(dev, DRV83XX_REG_OCP_CONTROL,
                     DRV83XX_OCP_OCPMODE_MASK,
                     DRV83XX_OCP_OCPMODE(mode));
  return OK;
}

/****************************************************************************
 * Name: drv83xx_get_ocp_mode
 ****************************************************************************/

int drv83xx_get_ocp_mode(FAR struct drv83xx_dev_s *dev, FAR int *mode)
{
  uint16_t ocp = 0;

  DEBUGASSERT(mode != NULL);

  drv83xx_read_reg(dev, DRV83XX_REG_OCP_CONTROL, &ocp);
  *mode = (int)((ocp & DRV83XX_OCP_OCPMODE_MASK) >>
                DRV83XX_OCP_OCPMODE_SHIFT);
  return OK;
}

/****************************************************************************
 * Name: drv83xx_csa_phase_cal
 ****************************************************************************/

int drv83xx_csa_phase_cal(FAR struct drv83xx_dev_s *dev,
                          FAR const struct drv83xx_csa_cal_s *cal)
{
  uint16_t set = 0;

  DEBUGASSERT(cal != NULL);

  if (cal->ch_a) set |= DRV83XX_CSACTL_CSA_CAL_A;
  if (cal->ch_b) set |= DRV83XX_CSACTL_CSA_CAL_B;
  if (cal->ch_c) set |= DRV83XX_CSACTL_CSA_CAL_C;

  drv83xx_modify_reg(dev, DRV83XX_REG_CSA_CONTROL,
                     DRV83XX_CSACTL_CSA_CAL_ALL, set);
  return OK;
}

/****************************************************************************
 * Name: drv83xx_calibration_all
 ****************************************************************************/

int drv83xx_calibration_all(FAR struct drv83xx_dev_s *dev, bool state)
{
  if (state)
    {
      drv83xx_modify_reg(dev, DRV83XX_REG_CSA_CONTROL,
                         0, DRV83XX_CSACTL_CSA_CAL_ALL);
    }
  else
    {
      drv83xx_modify_reg(dev, DRV83XX_REG_CSA_CONTROL,
                         DRV83XX_CSACTL_CSA_CAL_ALL, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: drv83xx_lock_regs
 ****************************************************************************/

int drv83xx_lock_regs(FAR struct drv83xx_dev_s *dev, int lock)
{
  uint16_t set = lock ? DRV83XX_GDHS_LOCK_LOCKED
                      : DRV83XX_GDHS_LOCK_UNLOCKED;

  drv83xx_modify_reg(dev, DRV83XX_REG_GATE_DRIVE_HS,
                     DRV83XX_GDHS_LOCK_MASK, set);
  return OK;
}
