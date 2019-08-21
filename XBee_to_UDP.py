"""
XBee to UDP adapter script
Author: Campbell McDiarmid
"""


########################################################################################################################
#
#                                                    IMPORTS
#
########################################################################################################################


import time
import threading
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.exception import InvalidPacketException, TimeoutException, InvalidOperatingModeException, XBeeException
from commonlib import device_finder, MAVQueue, Fifo, send_buffer_limit_rate
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil


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
    '0013A20040E2AB74': 'Worker #1',
    '0013A20040D68C32': 'Relay #1',
    '0013A20041520335': 'Navi #3',
}

########################################################################################################################
#
#                                                  FUNCTIONS/CLASSES
#
########################################################################################################################


class XBee2UDP(object):
    def __init__(self, connections, serial_port='/dev/ttyUSB0', baud_rate=XBEE_MAX_BAUD, **kwargs):
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

        # Initialize LUT for queues, MAVLink UDP sockets and stream parsers
        self.queue_in = {}
        self.queue_out = {}
        self.mav_socks = {}
        self.parsers = {}
        self.remote_xbees = {}

        # Initialize XBee device connection
        self.xbee = XBeeDevice(port=serial_port, baud_rate=baud_rate, **kwargs)
        while not self.xbee.is_open():
            try:
                self.xbee.open()
            except (InvalidPacketException, InvalidOperatingModeException, TimeoutException) as e:
                print(f'{e} raised while attempting to open XBee')
                time.sleep(3)

        print('XBee opened successfully.')
        self.main_running = False
        self.dev_running = {}
        self._udp_tx_closed = True
        self.network = None

    def start(self):
        """
        Starts all processes and threads in appropriate order
        """
        # Initialize network and discover devices (blocks other functionality and takes a while)
        self.network = self.xbee.get_network()
        self.network.set_discovery_timeout(5)
        self.network.start_discovery_process()
        self.main_running = True

        _loop = 0
        while self.network.is_discovery_running():
            print(f'\rDiscovering remote devices{"."*(_loop % 4):<4}', end='', flush=True)
            _loop += 1
            time.sleep(0.25)

        print(f'\nDiscovery complete!  {self.network.get_number_devices()} devices found')

        # Spawn UDP Rx threads for each connection
        for device in self.network.get_devices():
            self.new_remote_device(device)

        # Spawn UDP Tx thread
        _udp_tx_thread = threading.Thread(target=self._udp_tx_thread, daemon=True)
        _udp_tx_thread.start()

        # Once discovery has successfully completed, begin main loops
        _xbee_thread = threading.Thread(target=self._xbee_thread, daemon=True)
        _xbee_thread.start()

    def close(self):
        """
        Sets while-loop conditions to False for all threads, so each will run to completion and appropriate clean-up
        processes will be executed prior to the script terminating.
        """
        print(f'\nClosing script...')
        self.main_running = False
        devices_copy = self.dev_running.copy()
        for addr64 in devices_copy:
            self.del_remote_device(addr64)

    def new_remote_device(self, device: RemoteXBeeDevice):
        """
        Function for creating all of the necessary queues, parsers and connections associated with a new remote device
        :param device: New remote xbee device object
        """
        # Check whether detected device was specified during initialization
        addr64 = device.get_64bit_addr().address.hex().upper()

        if addr64 not in self.connections:
            ip, port = max(self.connections.values(),
                           key=lambda c: c[0] == LOCALHOST and c[1],
                           default=(LOCALHOST, UDP_IP))
            port += 1
            self.connections[addr64] = (ip, port)

        else:
            ip, port = self.connections[addr64]

        print(f'Assigned {REMOTE_DEVICE_IDS[addr64]} link to UDP {(ip, port)}')
        self.dev_running[addr64] = True
        self.mav_socks[addr64] = mavutil.mavudp(device=f'{ip}:{port}', input=False)
        self.queue_in[addr64] = MAVQueue()
        self.queue_out[addr64] = MAVQueue()
        self.parsers[addr64] = mavlink.MAVLink(Fifo())
        self.remote_xbees[addr64] = device
        _udp_rx_thread_x = threading.Thread(target=self._udp_rx_thread, args=(addr64,), daemon=True)
        _udp_rx_thread_x.start()

    def del_remote_device(self, addr64):
        """
        Delete remote XBee device if the connection has been lost.  This device will automatically added in and
        connection will be restored if it can send a message to the GCS XBee.

        :param addr64: String representation of 64 bit address
        """
        print(f'Deleting device {REMOTE_DEVICE_IDS[addr64]}')
        self.dev_running[addr64] = False
        time.sleep(0.02)

        for lut in (self.remote_xbees, self.queue_in, self.queue_out, self.mav_socks, self.parsers, self.dev_running):
            del lut[addr64]

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
                    clear self.queue_out[addr64] on the same line.
                b.  Send up to the oldest XBEE_PKT_MAX bytes of to the device.
                c.  Delete the bytes sent from outgoing through slicing.
                d.  Repeat b. and c. if outgoing != b''
        II.  TODO Check incoming broadcasts
        III. TODO Send any broadcasts to all devices if necessary
        IV.  Short sleep before next loop.
        """
        _bytes_in = 0
        _bytes_out = 0
        _loops = 0
        _start_time = time.time()
        _prev_print = time.time()
        _print_period = 0.5

        while self.main_running:
            # Service messages from vehicles (incoming/Rx)
            rx_packet = self.xbee.read_data()
            while rx_packet:
                addr64 = rx_packet.to_dict()['Sender: ']

                if addr64 not in self.queue_in:
                    self.new_remote_device(rx_packet.remote_device)

                try:
                    mav_msgs = self.parsers[addr64].parse_buffer(rx_packet.data)
                except mavlink.MAVError as e:
                    print(e)
                else:
                    if mav_msgs:
                        self.queue_in[addr64].extend(mav_msgs)
                        _bytes_in += len(rx_packet.data)
                rx_packet = self.xbee.read_data()

            # Service queues from GCS (outgoing/Tx)
            devices_copy = self.remote_xbees.copy()
            for addr64, device in devices_copy.items():
                outgoing = b''
                while self.queue_out[addr64]:
                    outgoing += bytes(self.queue_out[addr64].read().get_msgbuf())

                _bytes_out += len(outgoing)
                try:
                    send_buffer_limit_rate(self.xbee, device, outgoing, 230400)
                except XBeeException:
                    print(f'\nConnection lost with {REMOTE_DEVICE_IDS[addr64]}')
                    self.del_remote_device(addr64)

            # Wait between loops
            if time.time() - _prev_print >= _print_period:
                _prev_print = time.time()
                print(f'\rIN: {8*_bytes_in/(time.time() - _start_time):>10.2f}bps '
                      f'OUT: {8*_bytes_out/(time.time() - _start_time):>10.2f}bps LOOPS: {_loops}', end='', flush=True)
            _loops += 1
            time.sleep(0.0001)

        self.xbee.close()

    def _udp_rx_thread(self, addr64: str):
        """
        One thread is created for each UDP-XBee connection required, as socket.recvfrom() is a blocking function, and
        will wait until data has been received from QGroundControl (or other GCS software).  By doing this, new data
        that is forwarded on from the XBee to QGroundControl is not held up by the recvfrom function.

        :param addr64: Unique 64bit address associated with a remote XBee device, string representation
        """
        # Thread Main Loop
        while self.dev_running[addr64]:
            msg = self.mav_socks[addr64].recv_msg()
            if msg:
                self.queue_out[addr64].write(msg)
            time.sleep(0.01)

    def _udp_tx_thread(self):
        """
        UDP transmission thread that iterates through the queues of each remote XBee device by Node ID and sends any
        data directly along the associated UDP socket to QGroundControl or other GCS.
        """
        print(f'Started UDP Tx Thread')
        while self.main_running:
            for addr64 in self.remote_xbees:
                if self.queue_in[addr64]:
                    msg = self.queue_in[addr64].read()
                    self.mav_socks[addr64].write(msg.get_msgbuf())
            time.sleep(0.01)


def main():
    # TODO: Add command line argument passing
    uav_xbee_lut = {
        '0013A20040E2AB74': (LOCALHOST, 14554),
        '0013A20040D68C32': (LOCALHOST, 14555),
        '0013A20041520335': (LOCALHOST, 14556),
    }
    xb_port = device_finder('XBee')
    xb = XBee2UDP(uav_xbee_lut, serial_port=xb_port, baud_rate=XBEE_MAX_BAUD)
    xb.start()

    try:
        while True:
            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit):
        xb.close()
        del xb


if __name__ == '__main__':
    main()
