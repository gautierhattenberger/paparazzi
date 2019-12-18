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
 * @file modules/datalink/bitcraze/syslink_dl.h
 *
 * Syslink protocol handling and functionalities
 *
 */

#ifndef SYSLINK_DL_H
#define SYSLINK_DL_H

#include "modules/datalink/bitcraze/syslink.h"
#include "modules/datalink/bitcraze/crtp.h"
#include "pprzlink/pprzlink_device.h"

struct syslink_dl {
  // syslink structures
  syslink_parse_state state;
  syslink_message_t msg_rx;
  syslink_message_t msg_tx;

  // generic device to use pprzlink over syslink
  struct link_device device;

  // crazyflie state
  uint8_t rssi;
  bool charging;  ///< battery charging
  bool powered;   ///< USB powered
};

extern struct syslink_dl syslink;

/** Init function */
extern void syslink_dl_init(void);

/** Periodic function */
extern void syslink_dl_periodic(void);

/** Datalink event */
extern void syslink_dl_event(void);

#endif

