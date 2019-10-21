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
from commonlib import device_finder, MAVQueue, Fifo, send_buffer_limit_rate, \
    mav_rx_thread, queue_scheduled
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil


########################################################################################################################
#
#                                                    CONSTANTS
#
########################################################################################################################


# Localhost IP and arbitrarily defined based port
LOCALHOST = '127.0.0.1'
WIFI = '192.168.1.62'
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
RELAY_PX4 = 'LOCAL'

########################################################################################################################
#
#                                                  FUNCTIONS/CLASSES
#
########################################################################################################################


class XBee2UDP(object):
    def __init__(self, connections, serial_port='/dev/ttyUSB0',
    baud_rate=XBEE_MAX_BAUD, relay=False, **kwargs):
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

        if relay:
            px4_port = device_finder('FT232R USB UART')
            px4 = mavutil.mavserial(px4_port, PX4_COMPANION_BAUD, source_system=19, source_component=1)
            self.relay = px4
            self.new_remote_device(px4, relay=True)
            self.next = {k: time.time() + PX4_MAV_PERIODS[k] for k in PX4_MAV_PERIODS}

        else:
            self.relay = None

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
        _main_thread = threading.Thread(target=self._main_thread, daemon=True)
        _main_thread.start()

    def close(self):
        """
        Sets while-loop conditions to False for all threads, so each will run to completion and appropriate clean-up
        processes will be executed prior to the script terminating.
        """
        print(f'\nClosing script...')
        self.main_running = False
        devices_copy = self.dev_running.copy()
        for addr in devices_copy:
            self.del_remote_device(addr)

    def new_remote_device(self, device, relay=False):
        """
        Function for creating all of the necessary queues, parsers and connections associated with a new remote device
        :param device: New remote xbee device object
        """

        # Check whether detected device was specified during initialization
        if relay:
            ip = WIFI
            addr = RELAY_PX4
            self.queue_in[addr] = MAVQueue()
            self.parsers[RELAY_PX4] = mavlink.MAVLink(Fifo())

            _parse_thread = threading.Thread(target=mav_rx_thread, args=(device, self.queue_in[addr],), daemon=True)
            _parse_thread.start()

        else:
            addr = device.get_64bit_addr().address.hex().upper()
            ip = LOCALHOST
            self.queue_in[addr] = MAVQueue()
            self.queue_out[addr] = MAVQueue()
            self.parsers[addr] = mavlink.MAVLink(Fifo())
            self.remote_xbees[addr] = device

        if addr not in self.connections:
            _, port = max(self.connections.values(),
                           key=lambda c: c[0] == ip and c[1],
                           default=(ip, UDP_IP))
            port += 1
            self.connections[addr] = (ip, port)

        else:
            _, port = self.connections[addr]

        print(f'Assigned {REMOTE_DEVICE_IDS[addr]} link to UDP {(ip, port)}')

        self.dev_running[addr] = True
        self.mav_socks[addr] = mavutil.mavudp(device=f'{ip}:{port}', input=False)
        _udp_rx_thread_x = threading.Thread(target=self._udp_rx_thread, args=(addr,), daemon=True)
        _udp_rx_thread_x.start()

    def del_remote_device(self, addr):
        """
        Delete remote XBee device if the connection has been lost.  This device will automatically added in and
        connection will be restored if it can send a message to the GCS XBee.

        :param addr: String representation of 64 bit address
        """
        print(f'Deleting device {REMOTE_DEVICE_IDS[addr]}')
        self.dev_running[addr] = False
        time.sleep(0.02)

        for lut in (self.remote_xbees, self.queue_in, self.queue_out, self.mav_socks, self.parsers, self.dev_running):
            del lut[addr]

    def _main_thread(self):
        """
        Primary loop for servicing and passing data to the correct place:
        I.   Read messages from the XBee's Rx FIFO buffer, parse and place the in the appropriate queue.  New messages
             from undiscovered devices will automatically add them to the list of known devices, creating queues for each
        II.  Iterate through known devices, clear outgoing queue for each and attempt to transmit.  If there is an error
             in transmission this device will be deleted, along with all associated queues and UDP connections.
        III. (Optional) If Relay UAV deal with Relay PX4 IO
        IV.  TODO Check incoming broadcasts
        V.   TODO Send any broadcasts to all devices if necessary
        """
        # TODO: Probably best to have a moving average rather than overall aveage
        _bytes_in = 0
        _bytes_out = 0
        _loops = 0
        _start_time = time.time()
        _prev_print = time.time()
        _print_period = 0.5
        seq_counter = 0

        while self.main_running:
            # I. Service messages from vehicles (incoming/Rx)
            rx_packet = self.xbee.read_data()
            while rx_packet:
                addr = rx_packet.to_dict()['Sender: ']

                if addr not in self.queue_in:
                    self.new_remote_device(rx_packet.remote_device)

                # Check MAVLink message for errors
                try:
                    mav_msgs = self.parsers[addr].parse_buffer(rx_packet.data)
                except mavlink.MAVError as e:
                    print(e)
                else:
                    if mav_msgs:
                        self.queue_in[addr].extend(mav_msgs)
                        _bytes_in += len(rx_packet.data)
                rx_packet = self.xbee.read_data()

            # II. Service queues from GCS (outgoing/Tx)
            devices_copy = self.remote_xbees.copy()
            for addr, device in devices_copy.items():
                if addr == RELAY_PX4:
                    continue

                outgoing = self.queue_out[addr].read_bytes()
                # An XBee exception will be thrown if the connection has been lost
                try:
                    send_buffer_limit_rate(self.xbee, device, outgoing, 230400)
                except XBeeException:
                    print(f'\nConnection lost with {REMOTE_DEVICE_IDS[addr]}')
                    self.del_remote_device(addr)

                _bytes_out += len(outgoing)

            # III. If Relay
            if self.relay:
                priority_buffer = b''

                while priority_queue:
                    msg = self.queue_in[RELAY_PX4].read()
                    msg_bytes = replace_seq(msg, seq_counter)
                    priority_buffer += msg_bytes
                    seq_counter += 1
                    print(f'Priority message of type: {msg.get_type()}')

                queue_buffer, seq_counter  = queue_scheduled(seq_counter, self.next, self.relay)
                # mav_msgs = self.parsers[RELAY_PX4].parse_buffer(priority_buffer + queue_buffer)
                self.mav_socks[RELAY_PX4].write(priority_buffer + queue_buffer)

            # Wait between loops
            if time.time() - _prev_print >= _print_period:
                _prev_print = time.time()
                print(f'\rIN: {8*_bytes_in/(time.time() - _start_time):>10.2f}bps '
                      f'OUT: {8*_bytes_out/(time.time() - _start_time):>10.2f}bps LOOPS: {_loops}', end='', flush=True)
            _loops += 1
            time.sleep(0.001)

        self.xbee.close()

    def _udp_rx_thread(self, addr: str):
        """
        One thread is created for each UDP-XBee connection required, as socket.recvfrom() is a blocking function, and
        will wait until data has been received from QGroundControl (or other GCS software).  By doing this, new data
        that is forwarded on from the XBee to QGroundControl is not held up by the recvfrom function.

        :param addr: Unique 64bit address associated with a remote XBee device, string representation
        """
        # Thread Main Loop
        while self.dev_running[addr]:
            msg = self.mav_socks[addr].recv_msg()
            if msg:
                if addr == RELAY_PX4:
                    self.relay.write(msg.get_msgbuf())
                else:
                    self.queue_out[addr].write(msg)
            time.sleep(0.01)

    def _udp_tx_thread(self):
        """
        UDP transmission thread that iterates through the queues of each remote XBee device by Node ID and sends any
        data directly along the associated UDP socket to QGroundControl or other GCS.
        """
        print(f'Started UDP Tx Thread')
        while self.main_running:
            for addr in self.remote_xbees:
                if self.queue_in[addr]:
                    msg = self.queue_in[addr].read()
                    self.mav_socks[addr].write(msg.get_msgbuf())
            time.sleep(0.01)


def main(relay=False):
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
