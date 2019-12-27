#!/usr/bin/env python3
"""
Bridge a Crazyflie connected to a Crazyradio to the Ivy software bus
with support of PPRZLINK messages

Requires 'pip3 install cflib'

As the ESB protocol works using PTX and PRX (Primary Transmitter/Reciever)
modes. Thus, data is only recieved as a response to a sent packet.
So, we need to constantly poll the receivers for bidirectional communication.

@author: Dennis Shtatnov (densht@gmail.com)
         Gautier Hattenberger for Paparazzi UAV support

"""
# import struct
from os import path, getenv
import logging
import socket
import sys
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
# from cflib.crtp.crtpstack import CRTPPort

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.pprz_transport import PprzTransport
from pprzlink.message import PprzMessage
import pprzlink.messages_xml_map as messages_xml_map

CRTP_PORT_PPRZLINK = 9


# Only output errors from the logging framework
#logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.ERROR)


class RadioBridge:
    def __init__(self, link_uri, ac_id='^[0-9]+', msg_class='telemetry', verbose=False):
        """ Initialize and run with the specified link_uri """
        self.verbose = verbose

        # Ivy interface and stream parser
        self._ivy = IvyMessagesInterface("cf2ivy")
        self._transport = PprzTransport(msg_class)

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        if self.verbose:
            print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        self._cf._param_toc_updated_cb()

        # Bind to all messages from ac_id
        def _forward_to_cf(ac_id, msg):
            try:
                data = self._transport.pack_pprz_msg(0, msg) # sender_id 0 = GCS
                #print("forward msg", msg, data)
                for i in range(0, len(data), 30):
                    self._forward(data[i:(i+30)])
            except:
                if self.verbose:
                    print('Forward error for', ac_id)
        messages_datalink = messages_xml_map.get_msgs("datalink")
        for msg in messages_datalink:
            self._ivy.subscribe(_forward_to_cf, PprzMessage("datalink", msg))


    def shutdown(self):
        if self.verbose:
            print('closing cf2ivy interfaces')
        self._ivy.shutdown()
        self._cf.close_link()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        if self.verbose:
            print('Connected to %s' % link_uri)

        self._cf.packet_received.add_callback(self._got_packet)

    def _got_packet(self, pk):
        if pk.port == CRTP_PORT_PPRZLINK:
            for c in pk.data:
                if self._transport.parse_byte(bytes([c])):
                    (sender_id, _, _, msg) = self._transport.unpack()
                    if self.verbose:
                        print("Got message {} from {}".format(msg.name, sender_id))
                    # Forward message to Ivy bus
                    if self.is_connected:
                        self._ivy.send(msg, sender_id=sender_id)

    def _forward(self, data):
        pk = CRTPPacket()
        pk.port = CRTP_PORT_PPRZLINK
        pk.data = data
        self._cf.send_packet(pk)


    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        if self.verbose:
            print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description="Crazyradio link for paparazzi")
    parser.add_argument("--address", default=None, help="URI address of Crazyflie")
    parser.add_argument("--chanel", default='80', help="URI chanel of Crazyflie (full URI will be 'radio://0/%(default)/2M'")
    parser.add_argument("--uri", default=None, help="URI of Crazyflie (chanel option will not be effective)")
    parser.add_argument("--bus", default=None, help="Ivy bus. [default to system IVY bus]")
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
    args = parser.parse_args()

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.radiodriver.set_retries_before_disconnect(1500)
    cflib.crtp.radiodriver.set_retries(3)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Scan for Crazyflies and use the first one found
    if args.verbose:
        print('Scanning interfaces for Crazyflies...')
    if args.address is not None:
        address = int(sys.argv[2], 16)
    else:
        address = None # equivalent to default 0xE7E7E7E7E7
    available = cflib.crtp.scan_interfaces(address)
    if args.verbose:
        print('Crazyflies found:')
        for i in available:
            print(' ',i[0])

    if len(available) > 0:
        link_uri = None
        if args.uri is not None:
            link_uri = args.uri
        else:
            link_uri = 'radio://0/' + args.chanel + '/2M'

        # Start radio to ivy bridge
        bridge = RadioBridge(link_uri)

        # The Crazyflie lib doesn't contain anything to keep the application alive,
        # so this is where your application should do something. In our case we
        # are just waiting until we are disconnected.
        try:
            while bridge.is_connected:
                time.sleep(1)
        except KeyboardInterrupt:
            bridge.is_connected = False
            bridge.shutdown()
            time.sleep(1)
            sys.exit()
    else:
        if args.verbose:
            print("Leaving, no radio")
        sys.exit(1)

