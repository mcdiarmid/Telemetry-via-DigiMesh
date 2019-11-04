import platform
import time
import serial.tools.list_ports as list_ports
import struct
from pymavlink.generator.mavcrc import x25crc
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.exception import XBeeException


XBEE_MAX_BAUD = 230400
MAVLINK_SEQ_BYTE = 4


class Fifo(list):
    def write(self, data):
        self.__iadd__(data)
        return len(data)

    def read(self):
        return self.pop(0)


class MAVQueue(list):
    def write(self, mav_msg, priority=False):
        if priority:
            self.insert(0, mav_msg)
        else:
            self.append(mav_msg)
        return 1

    def read(self):
        return self.pop(0)

    def read_bytes(self):
        ret = b''
        while self:
            ret += bytes(self.read().get_msgbuf())
        return ret


def _device_finder_linux(name):
    while True:
        for comport in list_ports.comports():
            if comport.product:
                if name in comport.product:
                    print(f'{name} found')
                    return comport.device
        print(f'List of Comports: {list_ports.comports()}')
        print(f'Please insert {name} device')
        time.sleep(5)


def _device_finder_windows(name):
    while True:
        for comport in list_ports.comports():
            if comport.device == 'COM3':  # TODO
                print(f'{name} found')
                return comport.device
        print(f'List of Comports: {list_ports.comports()}')
        print(f'Please insert {name} device')
        time.sleep(5)


if platform.system() == 'Linux':
    device_finder = _device_finder_linux
elif platform.system() == 'Windows':
    device_finder = _device_finder_windows


def send_buffer_limit_rate(local_xbee: XBeeDevice, remote_xbee: RemoteXBeeDevice, buffer: bytes, max_bps: int):
    """
    Loop through and send a buffer of bytes in a more controlled method than previously intended.

    :param local_xbee: Transmitter XBee object
    :param remote_xbee: Remote Receiver XBee object
    :param buffer: Buffer containing bytes to be sent
    :param max_bps: Maximum transmission rate in bits per second to avoid large clumps of fast data
    """
    while buffer:
        pre = time.time()
        local_xbee.send_data(remote_xbee, buffer[:255])
        n_sent = len(buffer[:255]) * 8
        buffer = buffer[255:]
        while time.time() - pre < n_sent/max_bps:
            continue


def reconnect_blocker(local: XBeeDevice, remote: RemoteXBeeDevice):
    """
    Blocking function that awaits a message from a recently disconnected device.

    :param local: Local XBee radio (connected to computer via USB)
    :param remote: Remote XBee radio that a connection has been lost with
    """
    print(f'Link lost with coordinator')
    reconnected = False
    while not reconnected:
        try:
            _ = local.read_data_from(remote)
        except XBeeException:
            time.sleep(1)
        else:
            reconnected = True
    print(f'Link lost with coordinator')


def replace_seq(msg, seq):
    """
    from https://mavlink.io/en/about/overview.html

    uint8_t magic;              ///< protocol magic marker
    uint8_t len;                ///< Length of payload
    uint8_t incompat_flags;     ///< flags that must be understood
    uint8_t compat_flags;       ///< flags that can be ignored if not understood
    uint8_t seq;                ///< Sequence of packet
    uint8_t sysid;              ///< ID of message sender system/aircraft
    uint8_t compid;             ///< ID of the message sender component
    uint8_t msgid 0:7;          ///< first 8 bits of the ID of the message
    uint8_t msgid 8:15;         ///< middle 8 bits of the ID of the message
    uint8_t msgid 16:23;        ///< last 8 bits of the ID of the message
    uint8_t payload[max 255];   ///< A maximum of 255 payload bytes
    uint16_t checksum;          ///< X.25 CRC

    :param msg: MAVLink message from PX4
    :param seq: New sequence value
    :return: bytes object for the message buffer
    """
    data = msg.get_msgbuf()
    data[MAVLINK_SEQ_BYTE] = seq % 255
    cc = x25crc(bytes(data)[1:-2])
    if msg.crc_extra:
        cc.accumulate(struct.pack('B', msg.crc_extra))
    data[-2], data[-1] = cc.crc & 0xFF, cc.crc >> 8
    return bytes(data)


def queue_scheduled(seq_counter, next_times, px4, mavrate_lut):
    buffer = b''
    for mav_type in mavrate_lut:
        if time.time() >= next_times[mav_type]:
            next_times[mav_type] = time.time() + mavrate_lut[mav_type]

            if mav_type not in px4.messages:
                print(f'MAVLink message of type {mav_type} has not yet been received!')
                continue
            msg = px4.messages[mav_type]
            msg_bytes = replace_seq(msg, seq_counter)
            buffer += msg_bytes
            seq_counter += 1

    return buffer, seq_counter
