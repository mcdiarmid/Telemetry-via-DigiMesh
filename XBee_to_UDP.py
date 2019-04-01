import socket, serial, time, datetime, threading, os
from digi.xbee.devices import XBeeDevice


# Buffer values based off QGroundControl git in src/comm/UDPLink.cc, l. 298-299
KiB = 1024
BUFFER_LIMIT = 4095

# Localhost IP and arbirtarily defined based port
UDP_PORT = 14555
UDP_IP = '127.0.0.1'
SCRIPT_START = time.time()

# MAVLink constants
HEADER_LEN = 6
LEN_BYTE = 1
CRC_LEN = 2
START_BYTE = 0xfe

# Fucntions and Classes
class XBeeToUDP(XBeeDevice):
    def __init__(self, serialport='/dev/ttyUSB0', baud_rate=230400,
            udp=('127.0.0.1', 14555), **kwargs):

        # Initialize XBee device connection
        super().__init__(port=serialport, baud_rate=baud_rate, **kwargs)

        # UDP variables
        self.udp_ip, self.udp_port = udp
        self.udp_connections = {}  # Port: (b'\xfe\x42... queue_in, queue_out)
        self.sockets = {}


    def start(self):
        self.open()
        time.sleep(0.1)
        # Primary loop - deals with incoming and outgoing XBee data
        serial_thread = threading.Thread(target=self.primary_loop,
                                        daemon=True)
        serial_thread.start()


    def primary_loop(self):
        mesh = self.get_network()
        # mesh.set_discovery_timeout(5)
        loops = 0

        mesh.start_discovery_process()

        print("Discovering Devices...")
        while mesh.is_discovery_running():
            time.sleep(0.1)
        print("Finished: {}\n".format(mesh.get_devices()))

        while True:
            # Check for new packets from each remote XBee
            for device in mesh.get_devices():
                message =  self.read_data_from(device)
                if message is not None:
                    indata = bytes(message.data)
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
                        pkt_sent = self.send_data(device, outdata[:255])
                        self.print_msg(key, outgoing=outdata[:255])
                        outdata = outdata[255:]

            # End of while loop
            loops += 1
            time.sleep(0.001)


    def udp_thread(self, key):
        # Create socket and update port value for future new UDP connections
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((self.udp_ip, self.udp_port))
        self.sockets[key] = sock
        self.udp_port += 1

        print('Created UDP link for wireless connection with {}'.format(key))

        # Perpetual loop for sending and receiving data to and from UDP conn
        starttime = time.time()

        while True:
            # Check for new data from UDP conn
            outdata, _ = sock.recvfrom(KiB)  # TODO blocking by default
            self.udp_connections[key]['OUT'] += outdata
            time.sleep(0.05)

    def spawn_udp_thread(self, key):
        print(key)
        self.udp_connections[key] = dict(IN=b'', OUT=b'', PORT=self.udp_port)
        _thread = threading.Thread(target=self.udp_thread, args=(key,), daemon=True)
        _thread.start()

    @staticmethod
    def print_msg(key, incoming=b'', outgoing=b''):
        cols, rows = os.get_terminal_size()
        elapsed = time.time()-SCRIPT_START
        lhs = 6
        while elapsed >= 10:
            elapsed %= 10
            lhs += 1
        rhs = cols - lhs - 2
        bpl = rhs // 4
        line_format = '{' + f': <{lhs}' + '}| {' + f': <{rhs}' + '}\n'
        txt = '-' * cols + '\n'

        i = -1
        while incoming:
            if i == -1:
                line = line_format.format(f'{time.time()-SCRIPT_START:.2f}s', key)
            else:
                line = line_format.format(f'IN[{i}]', ', '.join(r'%02X' % c for c  in incoming[:bpl]))
                incoming = incoming[bpl:]
            i += 1
            txt += line

        i = 0
        while outgoing:
            line = line_format.format(f'OUT[{i}]', ', '.join(r'%02X' % c for c  in outgoing[:bpl]))
            outgoing = outgoing[bpl:]
            txt += line
            i += 1

        print(txt + '\n')


if __name__ == '__main__':
    # TODO: Add command line argument passing 
    xb = XBeeToUDP('/dev/ttyUSB0', 230400)
    xb.start()

    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        xb.close()
        print('Terminating all threads and closing all ports.\nExiting...')
