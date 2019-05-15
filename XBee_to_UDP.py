"""
XBee to UDP adapter script
Author: Campbell McDiarmid
"""


########################################################################################################################
#
#                                                    IMPORTS
#
########################################################################################################################


import socket
import time
import threading
from digi.xbee.devices import XBeeDevice
from commonlib import print_msg, device_finder


########################################################################################################################
#
#                                                    CONSTANTS
#
########################################################################################################################


# Localhost IP and arbitrarily defined based port
LOCALHOST = '127.0.0.1'
UDP_PORT = 14555
UDP_IP = LOCALHOST

# MAVLink constants
HEADER_LEN = 6
LEN_BYTE = 1
CRC_LEN = 2
START_BYTE = 0xFE

# Other serial device constants
XBEE_PKT_MAX = 0xFF
XBEE_MAX_BAUD = 230400
PX4_DEFAULT_BAUD = 115200
KiB = 1024
SER_BUF_LIMIT = 0xFFF
REMOTE_DEVICE_IDS = {
    '0013a20040d68c32': 'Worker #1',
    '0013a20041520335': 'Navi #3',
}


########################################################################################################################
#
#                                                  FUNCTIONS/CLASSES
#
########################################################################################################################


class XBee2UDP(object):
    def __init__(self, connections, serial_port='/dev/ttyUSB0', baud_rate=XBEE_MAX_BAUD, udp_timeout=3, **kwargs):
        """
        Version 2 of the XBee to UDP class

        :param connections: A list of tuples containing expected UDP connections and associated XBee node IDs
        :param serial_port: Device location
        :param baud_rate: Baud rate
        :param udp: udp connection defaults (ip, port)
        :param kwargs: Other arguments passed to the XBee device initializer
        """
        # Organize LUT for UDP connections
        if any(isinstance(connections, t) for t in (list, tuple, set)):
            self.connections = {key: (ip, port) for key, ip, port in connections}
        elif isinstance(connections, dict):
            self.connections = connections
        else:
            raise TypeError('Expected an iterable of tuples (key, ip, port), or dict of tuples key: (ip, port)')

        self.queue_in = {key: b'' for key in self.connections}
        self.queue_out = {key: b'' for key in self.connections}
        self.sockets = {}

        # Initialize XBee device connection
        self.xbee = XBeeDevice(port=serial_port, baud_rate=baud_rate, **kwargs)
        time.sleep(0.01)
        self.xbee.open()
        self.running = False
        self._udp_tx_closed = True
        self.network = None
        self.start_time = 0
        self.udp_timeout = udp_timeout

    def start(self):
        """
        Starts all processes and threads in appropriate order
        TODO daemon=True necessary?
        """
        # Initialize network and discover devices (blocks other functionality and takes a while)
        print('Discovering remote devices...')
        self.network = self.xbee.get_network()
        self.running = True
        self.network.start_discovery_process()
        while self.network.is_discovery_running():
            time.sleep(0.1)

        print(f'Discovery complete!  {self.network.get_number_devices()} devices found')

        # Spawn UDP Rx threads for each connection
        for device in self.network.get_devices():
            key = device.get_64bit_addr().address.hex()

            # Check whether detected device was specified during initialization
            if key not in self.connections:
                print(f'XBee Device not specified detected:\n'
                      f'\tNI = {device.get_node_id()}\n'
                      f'\tSH = {key[:-8]:0>8}, SL = {key[-8:]:0>8}\n')
                ip, port = max(self.connections.values(),
                               key=lambda c: c[0] == LOCALHOST and c[1],
                               default=(LOCALHOST, UDP_IP))
                port += 1
                self.connections[key] = (ip, port)
                print(f'Assigned device link to UDP {(ip, port)}')

            else:
                ip, port = self.connections[key]

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self.udp_timeout)
            sock.connect((ip, port))
            self.sockets[key] = sock
            _udp_rx_thread_x = threading.Thread(target=self._udp_rx_thread, args=(key,), daemon=True)
            _udp_rx_thread_x.start()

        # Spawn UDP Tx thread
        _udp_tx_thread = threading.Thread(target=self._udp_tx_thread, daemon=True)
        _udp_tx_thread.start()

        # Once discovery has successfully completed, begin main loop
        self.start_time = time.time()
        _service_thread = threading.Thread(target=self._xbee_thread, daemon=True)
        _service_thread.start()

    def close(self):
        """
        Sets while-loop conditions to False for all threads, so each will run to completion and appropriate clean-up
        processes will be executed prior to the script terminating.
        """
        self.running = False

    def _xbee_thread(self):
        """
        Primary loop for servicing and passing data to the correct place.
        Each loop iterates through all known devices in the network. Inside each loop:
        I.  for device in devices:
            1. Obtain 64 bit address of the device
            2. Receive packet:  TODO possibly change to service ALL packets in queue, similar to Tx.
                a.  Pop the oldest packet queued from that particular device
                b.  Obtain useful information if the packet is not empty
                c.  Convert from bytearray to bytes and place into a queue for UDP transmission to GCS software
            3. Transmit packet(s):  TODO possibly change to service ONE packet (or some N packets) in queue, like Rx.
                a.  Read queued data for transmission to the device to the variable: outgoing.
                    clear self.queue_out[key] on the same line.
                b.  Send up to the oldest XBEE_PKT_MAX bytes of to the device.
                c.  Delete the bytes sent from outgoing through slicing.
                d.  Repeat b. and c. if outgoing != b''
        II.  TODO Check incoming broadcasts
        III. TODO Send any broadcasts to all devices if necessary
        IV.  Short sleep before next loop.
        """
        while self.running:
            # Service explicit messages
            for device in self.network.get_devices():
                key = device.get_64bit_addr().address.hex()

                # Service new packets from the XBee
                rx_packet = self.xbee.read_data_from(device)
                if rx_packet is not None:
                    incoming = bytes(rx_packet.data)
                    self.queue_in[key] += incoming
                    print_msg(name=REMOTE_DEVICE_IDS[key], start=self.start_time, data=incoming, is_incoming=True)

                # Service new data from UDP connections
                outgoing, self.queue_out[key] = self.queue_out[key], b''
                while outgoing:
                    self.xbee.send_data(device, outgoing[:XBEE_PKT_MAX])
                    print_msg(name=REMOTE_DEVICE_IDS[key], start=self.start_time, data=outgoing, is_incoming=False)
                    outgoing = outgoing[XBEE_PKT_MAX:]

            # Wait between loops
            time.sleep(0.001)

        self.xbee.close()

    def _udp_rx_thread(self, key: str):
        """
        One thread is created for each UDP-XBee connection required, as socket.recvfrom() is a blocking function, and
        will wait until data has been received from QGroundControl (or other GCS software).  By doing this, new data
        that is forwarded on from the XBee to QGroundControl is not held up by the recvfrom function.

        :param key: Unique 64bit address associated with a remote XBee device, string representation
        """
        print(f'Started UDP Rx Thread for {REMOTE_DEVICE_IDS[key]}')
        # Thread Main Loop
        while self.running:
            try:
                data, _ = self.sockets[key].recvfrom(KiB)
                self.queue_out[key] += data
            except ConnectionRefusedError as e:
                print(f'{e}: Please reconnect {self.connections[key]} in GCS')
            except socket.timeout as e:
                print(f'{e}: Check XBee-PX4 script on {REMOTE_DEVICE_IDS[key]}')
            time.sleep(0.001)

        # Wait until the UDP Tx thread has terminated before closing UDP socket
        while not self._udp_tx_closed:
            time.sleep(0.001)

        self.sockets[key].close()

    def _udp_tx_thread(self):
        """
        UDP transmission thread that iterates through the queues of each remote XBee device by Node ID and sends any
        data directly along the associated UDP socket to QGroundControl or other GCS.
        """
        print(f'Started UDP Tx Thread')
        self._udp_tx_closed = False

        while self.running:
            for key in self.queue_in:
                if self.queue_in[key]:
                    try:
                        bytes_sent = self.sockets[key].send(self.queue_in[key])
                        self.queue_in[key] = self.queue_in[key][bytes_sent:]
                    except ConnectionRefusedError as e:
                        print(f'{e}: Please reconnect {self.connections[key]} in GCS')
            time.sleep(0.001)

        self._udp_tx_closed = True


def main():
    # TODO: Add command line argument passing
    uav_xbee_lut = {
        '0013a20040d68c32': (LOCALHOST, 14555),
        '0013a20041520335': (LOCALHOST, 14556),
    }
    xb_port = device_finder('XBee')
    xb = XBee2UDP(uav_xbee_lut, serial_port=xb_port, baud_rate=XBEE_MAX_BAUD)
    xb.start()

    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        xb.close()


if __name__ == '__main__':
    main()
