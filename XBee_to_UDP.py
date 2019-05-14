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
from commonlib import print_msg


########################################################################################################################
#
#                                                    CONSTANTS
#
########################################################################################################################


# Localhost IP and arbirtarily defined based port
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
            print(key, type(key), device.get_64bit_addr().address, type(device.get_64bit_addr().address))
            ip, port = self.connections[key]
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect((ip, port))
            self.sockets[key] = sock
            _udp_rx_thread_x = threading.Thread(target=self._udp_rx_thread, args=(key,), daemon=True)
            _udp_rx_thread_x.start()

        # Spawn UDP Tx thread
        _udp_tx_thread = threading.Thread(target=self._udp_tx_thread, daemon=True)
        _udp_tx_thread.start()

        # Once discovery has successfully completed, begin main loop
        self.start_time = time.time()
        _service_thread = threading.Thread(target=self.service_loop, daemon=True)
        _service_thread.start()

    def close(self):
        """
        Sets while-loop conditions to False for all threads, so each will run to completion and appropriate clean-up
        processes will be executed prior to the script terminating.
        """
        self.running = False

    def service_loop(self):
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
            data, _ = self.sockets[key].recvfrom(KiB)
            self.queue_out[key] += data
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
                    bytes_sent = self.sockets[key].send(self.queue_in[key])
                    self.queue_in[key] = self.queue_in[key][bytes_sent:]
            time.sleep(0.001)

        self._udp_tx_closed = True


class XBeeToUDP(XBeeDevice):
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=XBEE_MAX_BAUD,
            udp=(UDP_IP, UDP_PORT), **kwargs):
        """

        :param serial_port:
        :param baud_rate:
        :param udp:
        :param kwargs:
        """

        # Initialize XBee device connection
        super().__init__(port=serial_port, baud_rate=baud_rate, **kwargs)

        # UDP variables
        self.udp_ip, self.udp_port = udp
        self.udp_connections = {}  # Port: (b'\xfe\x42... queue_in, queue_out)
        self.sockets = {}
        self.running = True
        self.starttime = time.time()

    def start(self):
        """
        TODO
        """
        self.open()
        time.sleep(0.1)
        # Primary loop - deals with incoming and outgoing XBee data
        serial_thread = threading.Thread(target=self.primary_loop, daemon=True)
        serial_thread.start()

    def close(self):
        """
        TODO
        """
        # Set while loop conditions to False for all threads finish executing
        self.running = False

    def primary_loop(self):
        """
        TODO
        """
        # Discover devices on the mesh network
        mesh = self.get_network()
        mesh.start_discovery_process()

        print("Discovering Devices...")
        while mesh.is_discovery_running():
            time.sleep(0.1)
        print("Finished: {}\n".format(mesh.get_devices()))

        bytes_recv = 0
        bytes_sent = 0
        recv_start = time.time()

        while self.running:
            # Check for new packets from each remote XBee
            for device in mesh.get_devices():
                message = self.read_data_from(device)
                if message is not None:
                    indata = bytes(message.data)
                    bytes_recv += len(indata)
                    key = device.get_64bit_addr().address.hex()

                    if key not in self.udp_connections:
                        # Add new queue and spawn new UDP thread
                        self.spawn_udp_thread(key)
                        time.sleep(0.001)

                    queued_data = self.udp_connections[key]['IN'] + indata
                    sent = self.sockets[key].send(queued_data)
                    print_msg(key, self.starttime, data=queued_data[:sent], is_incoming=True)
                    self.udp_connections[key]['IN'] = queued_data[sent:]

                    time_elapsed = time.time() - recv_start
                    in_rate = 8 * bytes_recv / time_elapsed / 1000
                    print('Recv: {0:.2f}kbps'.format(in_rate))

            # TODO Check general broadcasts

            # Check for new items in the outgoing queue
            for device in mesh.get_devices():
                key = device.get_64bit_addr().address.hex()

                if key not in self.udp_connections:
                    self.spawn_udp_thread(key)

                outdata = self.udp_connections[key]['OUT']
                if outdata:
                    self.udp_connections[key]['OUT'] = self.udp_connections[key]['OUT'][len(outdata):]
                    while outdata:
                        pkt_sent = self.send_data(device, outdata[:XBEE_PKT_MAX])
                        bytes_sent += len(outdata[:XBEE_PKT_MAX])
                        print_msg(key, self.starttime, data=outdata[:XBEE_PKT_MAX], is_incoming=False)
                        outdata = outdata[XBEE_PKT_MAX:]

                    time_elapsed = time.time() - recv_start
                    out_rate = 8 * bytes_sent / time_elapsed / 1000
                    print('Sent: {0:.2f}kbps'.format(out_rate))

            # End of while loop
            time.sleep(0.001)

        # Close XBee and all UDP Sockets
        super().close()
        time.sleep(1)
        for key in self.udp_connections:
            self.udp_connections[key].close()
            print('Exiting thread for {}'.format(key))

    def udp_thread(self, key):
        """
        TODO
        :param key:
        :return:
        """
        # Create socket and update port value for future new UDP connections
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((self.udp_ip, self.udp_port))
        self.sockets[key] = sock
        self.udp_port += 1

        print('Created UDP link for wireless connection with {}'.format(key))

        # Perpetual loop for sending and receiving data to and from UDP conn
        while self.running:
            # Check for new data from UDP conn
            outdata, _ = sock.recvfrom(KiB)  # TODO blocking by default
            self.udp_connections[key]['OUT'] += outdata
            time.sleep(0.05)

    def spawn_udp_thread(self, key):
        """

        :param key:
        :return:
        """
        print(key)
        self.udp_connections[key] = dict(IN=b'', OUT=b'', PORT=self.udp_port)
        _thread = threading.Thread(target=self.udp_thread, args=(key,), daemon=True)
        _thread.start()


def main():
    # TODO: Add command line argument passing
    # xb = XBeeToUDP('/dev/ttyUSB0', XBEE_MAX_BAUD)
    uav_xbee_lut = {
        '0013a20040d68c32': ('127.0.0.1', 14555),
    }
    xb = XBee2UDP(uav_xbee_lut, serial_port='/dev/ttyUSB0', baud_rate=XBEE_MAX_BAUD)
    xb.start()

    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        xb.close()


if __name__ == '__main__':
    main()
