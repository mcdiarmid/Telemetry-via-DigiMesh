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
        time.sleep(0.01)

        self.running = False
        self.network = None
        self.start_time = 0

    def start(self):
        # Initialize network and discover devices (blocks other functionality and takes a while)
        self.network = self.xbee.get_network()

        def callback(remote):
            print(f'Discovered: {remote.get_node_id()}')

        self.network.add_device_discovered_callback(callback)
        self.network.start_discovery_process()
        while self.network.is_discovery_running():
            time.sleep(0.1)

        # Spawn UDP threads for each connection
        for node_id in self.connections:
            if node_id in self.network.get_device_by_node_id(node_id):
                ip, port = self.connections[node_id]
                _thread = threading.Thread(target=self._udp_rx_thread, args=(node_id, ip, port), daemon=True)
                _thread.start()
            else:
                print(f'Device {node_id} not found')

        # Once discovery has successfully completed, begin main loop
        self.running = True
        self.start_time = time.time()
        _thread = threading.Thread(target=self.service_loop, daemon=True)
        _thread.start()

    def close(self):
        self.running = False

    def service_loop(self):
        while self.running:
            # Iterate through devices
            for device in self.network.get_devices():
                node_id = device.get_node_id()

                # Service new packets from the XBee
                rx_packet = self.xbee.read_data_from(device)
                if rx_packet is not None:
                    incoming = bytes(rx_packet.data)
                    self.queue_in[node_id] += incoming

                # Service new data from UDP connections
                outgoing, self.queue_out[node_id] = self.queue_out[node_id], b''
                while outgoing:
                    tx_packet = self.xbee.send_data(device, outgoing[:XBEE_PKT_MAX])
                    outgoing = outgoing[XBEE_PKT_MAX:]

            # Check general broadcasts TODO

            # Wait between loops
            time.sleep(0.001)

        # Tidy up
        for sock in self.sockets:
            sock.close()
        self.xbee.close()

    def _udp_rx_thread(self, node_id: str, ip: str, port: int):
        """
        
        :param node_id: Unique name associated with a remote XBee device, string representation
        :param ip: IP address (typically "127.0.0.1"), string representation
        :param port: Port number, integer representation
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((ip, port))
        self.sockets[node_id] = sock

        while self.running:
            data = sock.recvfrom(KiB)
            self.queue_out[node_id] += data
            time.sleep(0.001)

    def _udp_tx_thread(self):
        """
        UDP transmission thread that iterates through the queues of each remote XBee device by Node ID and sends any
        data directly along the associated UDP socket to QGroundControl or other GCS.
        """
        while self.running:
            for node_id in self.queue_in:
                if self.queue_in[node_id] and self.sockets[node_id]:
                    bytes_sent = self.sockets[node_id].send(self.queue_in[node_id])
                    self.queue_in[node_id] = self.queue_in[node_id][bytes_sent:]
            time.sleep(0.001)


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

        :return:
        """
        self.open()
        time.sleep(0.1)
        # Primary loop - deals with incoming and outgoing XBee data
        serial_thread = threading.Thread(target=self.primary_loop, daemon=True)
        serial_thread.start()

    def close(self):
        """

        :return:
        """
        # Set while loop conditions to False for all threads finish executing
        self.running = False

    def primary_loop(self):
        """

        :return:
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
                    print_msg(key, self.starttime, incoming=queued_data[:sent])
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
                        print_msg(key, self.starttime, outgoing=outdata[:XBEE_PKT_MAX])
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


if __name__ == '__main__':
    # TODO: Add command line argument passing
    xb = XBeeToUDP('/dev/ttyUSB0', XBEE_MAX_BAUD)
    xb.start()

    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        xb.close()
