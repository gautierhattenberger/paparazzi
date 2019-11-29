/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/bmi088_i2c.c
 *
 * Driver for the BMI088 using I2C.
 *
 */

#include "peripherals/bmi088_i2c.h"

void bmi088_i2c_init(struct Bmi088_I2c *bmi, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  bmi->i2c_p = i2c_p;

  /* slave address */
  bmi->i2c_trans.slave_addr = addr;
  /* set inital status: Success or Done */
  bmi->i2c_trans.status = I2CTransDone;
  bmi->acc_trans.status = I2CTransDone;

  /* set default BMI088 config options */
  bmi088_set_default_config(&(bmi->config));

  bmi->gyro_available = false;
  bmi->accel_available = false;
  bmi->config.initialized = false;
  bmi->config.init_status = BMI088_CONF_UNINIT;
}


static void bmi088_i2c_write_to_reg(void *bmi, uint8_t _reg, uint8_t _val)
{
  struct Bmi088_I2c *bmi_i2c = (struct Bmi088_I2c *)(bmi);
  bmi_i2c->i2c_trans.buf[0] = _reg;
  bmi_i2c->i2c_trans.buf[1] = _val;
  i2c_transmit(bmi_i2c->i2c_p, &(bmi_i2c->i2c_trans), bmi_i2c->i2c_trans.slave_addr, 2);
}

// Configuration function called once before normal use
void bmi088_i2c_start_configure(struct Bmi088_I2c *bmi)
{
  if (bmi->config.init_status == BMI088_CONF_UNINIT) {
    bmi->config.init_status++;
    if (bmi->i2c_trans.status == I2CTransSuccess || bmi->i2c_trans.status == I2CTransDone) {
      bmi088_send_config(bmi088_i2c_write_to_reg, (void *)bmi, &(bmi->config));
    }
  }
}

void bmi088_i2c_read(struct Bmi088_I2c *bmi)
{
  if (bmi->config.initialized && bmi->i2c_trans.status == I2CTransDone) {
    /* read gyro */
    bmi->i2c_trans.buf[0] = BMI088_GYRO_INT_STAT_1;
    i2c_transceive(bmi->i2c_p, &(bmi->i2c_trans), bmi->i2c_trans.slave_addr, 1, 9);
    /* read accel */
    bmi->acc_trans.buf[0] = BMI088_ACC_INT_STAT_1;
    i2c_transceive(bmi->i2c_p, &(bmi->i2c_trans), bmi->i2c_trans.slave_addr, 1, 12);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void bmi088_i2c_event(struct Bmi088_I2c *bmi)
{
  if (bmi->config.initialized) {
    // Check gyro read
    if (bmi->i2c_trans.status == I2CTransFailed) {
      bmi->i2c_trans.status = I2CTransDone;
    } else if (bmi->i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      if (bit_is_set(bmi->i2c_trans.buf[0], 7)) {
        // new data
        bmi->data_rates.rates.p = Int16FromBuf(bmi->i2c_trans.buf, 7);
        bmi->data_rates.rates.q = Int16FromBuf(bmi->i2c_trans.buf, 5);
        bmi->data_rates.rates.r = Int16FromBuf(bmi->i2c_trans.buf, 3);
        bmi->gyro_available = true;
      }
      bmi->i2c_trans.status = I2CTransDone;
    }
    // Check accel read
    if (bmi->acc_trans.status == I2CTransFailed) {
      bmi->acc_trans.status = I2CTransDone;
    } else if (bmi->acc_trans.status == I2CTransSuccess) {
      // Successfull reading
      if (bit_is_set(bmi->acc_trans.buf[0], 7)) {
        // new data
        bmi->data_accel.vect.x = Int16FromBuf(bmi->acc_trans.buf, 10);
        bmi->data_accel.vect.y = Int16FromBuf(bmi->acc_trans.buf, 8);
        bmi->data_accel.vect.z = Int16FromBuf(bmi->acc_trans.buf, 6);
        bmi->accel_available = true;
      }
      bmi->i2c_trans.status = I2CTransDone;
    }
  } else if (bmi->config.init_status != BMI088_CONF_UNINIT) { // Configuring but not yet initialized
    switch (bmi->i2c_trans.status) {
      case I2CTransFailed:
        bmi->config.init_status--; // Retry config (TODO max retry)
        /* Falls through. */
      case I2CTransSuccess:
      case I2CTransDone:
        bmi088_send_config(bmi088_i2c_write_to_reg, (void *)bmi, &(bmi->config));
        if (bmi->config.initialized) {
          bmi->i2c_trans.status = I2CTransDone;
        }
        break;
      default:
        break;
    }
  }
}

