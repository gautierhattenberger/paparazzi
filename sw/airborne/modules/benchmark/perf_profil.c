/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/benchmark/perf_profil.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Performance profiling tool
 */

#include "modules/benchmark/perf_profil.h"
#include "mcu_periph/sys_time.h"
#include "modules/loggers/sdlog_chibios.h"
#include <hal.h>
#include <ch.h>

// RTC2US(STM32_SYSCLK, chSysGetRealtimeCounterX)

void perf_profil_log(char * msg)
{
  if (pprzLogFile != -1) {
    uint32_t t = chSysGetRealtimeCounterX();
    sdLogWriteLog(pprzLogFile, "PPT %s %lu\n", msg, t);
  }
}

void perf_profil_log_time(char * msg, uint32_t t)
{
  if (pprzLogFile != -1) {
    sdLogWriteLog(pprzLogFile, "PPT %s %lu\n", msg, t);
  }
}

#ifndef PERF_PROFIL_EVENT_MAX
#define PERF_PROFIL_EVENT_MAX 1000
#endif

struct perf_struct {
  uint32_t nb_event;
  uint32_t nb_over;
  uint32_t t_start;
  uint32_t dt_start;
  uint32_t dt_end;
  uint32_t dt_min;
  uint32_t dt_max;
  double dt_acc;
};

#define PERF_STRUCT_INIT(_s) {  \
  _s.nb_event = 0;              \
  _s.nb_over = 0;               \
  _s.t_start = 0;               \
  _s.dt_start = 0;              \
  _s.dt_end = 0;                \
  _s.dt_min = 0xFFFFFFFF;       \
  _s.dt_max = 0;                \
  _s.dt_acc = 0.;               \
}

static struct perf_struct perf_array[PERF_STRUCT_NB];

void perf_profil_init(void)
{
  for (int i = 0; i < PERF_STRUCT_NB; i++) {
    PERF_STRUCT_INIT(perf_array[i]);
  }
}

void perf_profil_event_start(int idx)
{
  perf_array[idx].dt_start = chSysGetRealtimeCounterX();
  if (perf_array[idx].nb_event == 0) {
    perf_array[idx].t_start = perf_array[idx].dt_start;
  }
}

void perf_profil_event_end(int idx, char * msg, uint32_t freq)
{
  perf_array[idx].dt_end = chSysGetRealtimeCounterX();
  perf_array[idx].nb_event++;
  uint32_t dt = perf_array[idx].dt_end - perf_array[idx].dt_start;
  perf_array[idx].dt_acc += (double)(RTC2US(STM32_SYSCLK, dt));
  if (dt < perf_array[idx].dt_min) {
    perf_array[idx].dt_min = dt;
  }
  if (dt > perf_array[idx].dt_max) {
    perf_array[idx].dt_max = dt;
  }
  if (dt > US2RTC(STM32_SYSCLK, (1000000U/freq))) {
    perf_array[idx].nb_over++; // dt is over the polling inverval (1/CH_CFG_ST_FREQUENCY sec)
  }
  if (perf_array[idx].nb_event >= PERF_PROFIL_EVENT_MAX) {
    sdLogWriteLog(pprzLogFile, "PPTE %s %lu %lu %lu %lu %lu %.2f\n", msg,
        perf_array[idx].nb_event, perf_array[idx].nb_over,
        perf_array[idx].dt_end - perf_array[idx].t_start,
        perf_array[idx].dt_min,
        perf_array[idx].dt_max,
        perf_array[idx].dt_acc);
    perf_array[idx].nb_event = 0;
    perf_array[idx].nb_over = 0;
    perf_array[idx].dt_min = 0xFFFFFFFF;
    perf_array[idx].dt_max = 0;
    perf_array[idx].dt_acc = 0.;
  }
}

