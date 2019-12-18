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
 * @file modules/datalink/bitcraze/syslink_dl.c
 *
 * Syslink protocol handling and functionalities
 *
 */

#include "modules/datalink/bitcraze/syslink_dl.h"
#include "subsystems/electrical.h"
#include "mcu_periph/uart.h"
#include <string.h>

struct syslink_dl syslink;

/** Init function */
void syslink_dl_init(void)
{
  syslink_parse_init(&syslink.state);
}

void syslink_dl_periodic(void)
{
  // LED blinking (charge) ?
}

/**
 * Handle battery message
 */
static void handle_battery(syslink_message_t *msg)
{
  if (msg->length != 9) {
    return;
  }

  // check flag
  uint8_t flags = msg->data[0];
  syslink.charging = (bool) (flags & 1);
  syslink.powered = (bool) (flags & 2);

  // update voltage
  float vbat;
  memcpy(&vbat, &msg->data[1], sizeof(float));
  if (syslink.powered && vbat > 3.f) {
    electrical.vsupply = vbat; // remove 0 reading when powered on USB ?
  } else if (!syslink.powered) {
    electrical.vsupply = vbat; // running on battery
  }
}

/**
 * Handle raw datalink
 *
 * From Bitcraze documentation:
 *
 * This packet carries the raw radio packet. The NRF51 acts as a radio bridge.
 * Because the NRF51 does not have much memory and the STM32 is capable of bursting
 * a lot of data a flow control rules has been made: The STM32 is allowed to send
 * a RADIO_RAW packet only when one RADIO_RAW packet has been received.
 *
 * The NRF51 is regularly sending CRTP NULL packet or empty packets to the STM32
 * to get the communication working both ways.
 *
 * Note: So far RADIO_RAW is the only syslink packet that has flow control constrain,
 * all other packets can be sent full duplex at any moment.
 */
static void handle_raw(syslink_message_t *msg)
{
  crtp_message_t *c = (crtp_message_t *) &msg->length;

  if (CRTP_NULL(*c)) {
    if (c->size >= 3) {
      //handle_bootloader(sys);
    }
  }
  else if (c->port == CRTP_PORT_COMMANDER) {
    //crtp_commander *cmd = (crtp_commander *) &c->data[0];
    // TODO set RC from cmd message
  }
  else if (c->port == CRTP_PORT_PPRZLINK) {
    // TODO send to pprzlink parser
  }
  else {
    // TODO ACK for other messages ?
  }

  // TODO send next raw message


}

static void handle_radio(syslink_message_t *msg)
{
  if (msg->type == SYSLINK_RADIO_RSSI) {
		uint8_t rssi = msg->data[0]; // Between 40 and 100 meaning -40 dBm to -100 dBm
    syslink.rssi = 140 - rssi * 100 / (100 - 40);
	}
  else if (msg->type == SYSLINK_RADIO_CHANNEL) {
    // ack radio channel
  }
  else if (msg->type == SYSLINK_RADIO_DATARATE) {
    // ack radio datarate
  }
  else if (msg->type == SYSLINK_RADIO_ADDRESS) {
    // ack radio address
  }
}


/** New RX message
 */
static void handle_new_msg(syslink_message_t *msg)
{
	if (msg->type == SYSLINK_PM_ONOFF_SWITCHOFF) {
		// power button is hit
    // don't do anything for now ?
	}
  else if (msg->type == SYSLINK_PM_BATTERY_STATE) {
    handle_battery(msg);
	}
  else if (msg->type == SYSLINK_RADIO_RAW) {
		handle_raw(msg);
	}
  else if ((msg->type & SYSLINK_GROUP) == SYSLINK_RADIO) {
		handle_radio(msg);
	}
  else if ((msg->type & SYSLINK_GROUP) == SYSLINK_OW) {
    // one-wire memory access
    // don't do anything for now
	}
  else {
    // handle errors ?
	}

  // TODO prepare next message to send ?
}

/** Datalink event */
void syslink_dl_event(void)
{
  while (uart_char_available(&(SYSLINK_DEV))) {
    uint8_t ch = uart_getch(&(SYSLINK_DEV));
    if (syslink_parse_char(&syslink.state, ch, &syslink.msg_rx)) {
      handle_new_msg(&syslink.msg_rx);
    }
  }
}


