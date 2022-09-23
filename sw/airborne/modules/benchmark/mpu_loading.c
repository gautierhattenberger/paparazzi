/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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

/** @file "modules/benchmark/mpu_loading.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Simulate MCU loading with periodic active wait
 */

#include "modules/benchmark/mpu_loading.h"
#include "mcu_periph/sys_time.h"

void mcu_load_0_1Hz(void)
{
  // your periodic code here.
  // freq = 0.1 Hz
  sys_time_usleep(400);
}

void mcu_load_1Hz(void)
{
  // your periodic code here.
  // freq = 1.0 Hz
  sys_time_usleep(400);
}

void mcu_load_5Hz(void)
{
  // your periodic code here.
  // freq = 5.0 Hz
  sys_time_usleep(400);
}

void mcu_load_10Hz(void)
{
  // your periodic code here.
  // freq = 10.0 Hz
  sys_time_usleep(400);
}

void mcu_load_25Hz(void)
{
  // your periodic code here.
  // freq = 25.0 Hz
  sys_time_usleep(400);
}


