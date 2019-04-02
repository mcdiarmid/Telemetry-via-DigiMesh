import socket, serial, time, datetime, threading, os
from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import TimeoutException


# Localhost IP and arbirtarily defined based port
LOCALHOST = '127.0.0.1'
UDP_PORT = 14555
UDP_IP = LOCALHOST
SCRIPT_START = time.time()

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


# Fucntions and Classes
class XBeeToUDP(XBeeDevice):
    def __init__(self, serialport='/dev/ttyUSB0', baud_rate=XBEE_MAX_BAUD,
            udp=(UDP_IP, UDP_PORT), **kwargs):

        # Initialize XBee device connection
        super().__init__(port=serialport, baud_rate=baud_rate, **kwargs)

        # UDP variables
        self.udp_ip, self.udp_port = udp
        self.udp_connections = {}  # Port: (b'\xfe\x42... queue_in, queue_out)
        self.sockets = {}
        self.running = True


    def start(self):
        self.open()
        time.sleep(0.1)
        # Primary loop - deals with incoming and outgoing XBee data
        serial_thread = threading.Thread(target=self.primary_loop,
                                        daemon=True)
        serial_thread.start()

    def close(self):
        super().close()
        self.running = False


    def primary_loop(self):
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
        try:
            while self.running:
                # Check for new packets from each remote XBee
                for device in mesh.get_devices():
                    message =  self.read_data_from(device)
                    if message is not None:
                        indata = bytes(message.data)
                        bytes_recv += len(indata)
                        key = device.get_64bit_addr().address.hex()

                        if key not in self.udp_connections:
                            # Add new queue and spawn new UDP thread
                            self.spawn_udp_thread(key)
                            time.sleep(0.001)

                        queued_data = self.udp_connections[key]['IN'] + indata
                        # TODO: Semaphore/Mutex begin
                        sent = self.sockets[key].send(queued_data)
                        # TODO: Semaphore/Mutex end
                        self.print_msg(key, incoming=queued_data[:sent])
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

                    # TODO: Semaphore/mutex?
                    outdata = self.udp_connections[key]['OUT']
                    if outdata:
                        self.udp_connections[key]['OUT'] = self.udp_connections[key]['OUT'][len(outdata):]
                        while outdata:
                            pkt_sent = self.send_data(device, outdata[:XBEE_PKT_MAX])
                            bytes_sent += len(outdata[:XBEE_PKT_MAX])
                            self.print_msg(key, outgoing=outdata[:XBEE_PKT_MAX])
                            outdata = outdata[XBEE_PKT_MAX:]

                        time_elapsed = time.time() - recv_start
                        out_rate = 8 * bytes_sent / time_elapsed / 1000
                        print('Sent: {0:.2f}kbps'.format(out_rate))

                # End of while loop
                time.sleep(0.001)

        except TimeoutException:
                print('Exiting main loop')


    def udp_thread(self, key):
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

        sock.close()
        print('Exiting thread for {}'.format(key))

    def spawn_udp_thread(self, key):
        print(key)
        self.udp_connections[key] = dict(IN=b'', OUT=b'', PORT=self.udp_port)
        _thread = threading.Thread(target=self.udp_thread, args=(key,), daemon=True)
        _thread.start()

    @staticmethod
    def print_msg(key, incoming=b'', outgoing=b''):
        # Formats incoming and outgoing messages to fit the terminal window
        cols, rows = os.get_terminal_size()
        elapsed = time.time()-SCRIPT_START
        lhs = 6  # Left hand side width in monospace unicode characters

        while elapsed >= 10:  # Increase LHS for each digit of elapsed time
            elapsed %= 10
            lhs += 1

        rhs = cols - lhs - 2  # Right hand side width
        bpl = rhs // 4  # Bytes displayed per line
        brkline = '-' * cols + '\n'  # Line of hyphens
        line_format = '{' + f': <{lhs}' + '}| {' + f': <{rhs}' + '}\n'
        txt = brkline + line_format.format(f'{time.time()-SCRIPT_START:.2f}s', key)
        sep = ', '

        for direction, data in (('IN', incoming), ('OUT', outgoing)):
            nlines, rem = divmod(len(data), bpl)
            nlines += rem > 0
            for i in range(nlines):
                linedata = sep.join('%02X' % c for c  in data[i*bpl:(i+1)*bpl])
                line = line_format.format(f'{direction}[{i}]', linedata)
                txt += line
            if nlines:
                txt += brkline

        print(txt)


if __name__ == '__main__':
    # TODO: Add command line argument passing
    xb = XBeeToUDP('/dev/ttyUSB0', XBEE_MAX_BAUD)
    xb.start()

    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        xb.close()
