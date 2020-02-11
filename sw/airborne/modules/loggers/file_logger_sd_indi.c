/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger_sd_indi.h"

#include "modules/loggers/sdlog_chibios.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/commands.h"
#include "state.h"

static uint32_t counter;

/** Start the file logger and open a new file */
void file_logger_sd_indi_start(void)
{
  counter = 0;

  if (pprzLogFile != -1) {
    sdLogWriteLog(pprzLogFile,
      "counter,gyro_p,gyro_q,gyro_r,ax,ay,az,COMMAND_THROTTLE,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW\n"
      //"counter,gyro_p,gyro_q,gyro_r,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,accel_z\n"
    );
  }
}


/** Log the values to a csv file */
void file_logger_sd_indi_periodic(void)
{
  if (pprzLogFile == -1) {
    return;
  }

  //struct FloatRates *rates = stateGetBodyRates_f();
  struct Int32Rates *rates = stateGetBodyRates_i();
  struct Int32Vect3 *accel_body = stateGetAccelBody_i();
  //float accelz = ACCEL_FLOAT_OF_BFP(accel_body->z);

  sdLogWriteLog(pprzLogFile,
      //"%lu,%.5f,%.5f,%.5f,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
      "%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
      //"%lu,%.5f,%.5f,%.5f,%ld,%ld,%ld,%d,%d,%d\n",
      counter,
      rates->p,
      rates->q,
      rates->r,
      accel_body->x,
      accel_body->y,
      accel_body->z,
      //commands[COMMAND_THROTTLE],
      //commands[COMMAND_ROLL],
      //commands[COMMAND_PITCH]
      stabilization_cmd[COMMAND_THRUST],
      stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH],
      stabilization_cmd[COMMAND_YAW]
      //accelz
      );

  counter++;
}
