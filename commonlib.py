import os
import platform
import time
import serial.tools.list_ports as list_ports
import struct
from pymavlink.generator.mavcrc import x25crc
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice


MAVLINK_SEQ_BYTE = 4


class Fifo(list):
    def write(self, data):
        self.__iadd__(data)
        return len(data)

    def read(self):
        return self.pop(0)


class MAVQueue(list):
    def write(self, mav_msg):
        super().append(mav_msg)
        return 1

    def read(self):
        return super().pop(0)


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
            if comport.device == 'COM3':  # TODO Ugly code but temporary
                print(f'{name} found')
                return comport.device
        print(f'List of Comports: {list_ports.comports()}')
        print(f'Please insert {name} device')
        time.sleep(5)


if platform.system() == 'Linux':
    device_finder = _device_finder_linux
elif platform.system() == 'Windows':
    device_finder = _device_finder_windows


def write_buffer_log(logname, buffer):
    with open(logname, 'w') as f:
        line = ''
        for i, b in enumerate(buffer):
            line += f'{b:02X}'

            if not (i+1) % 25:
                _ = f.write(line + '\n')
                line = ''


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


def reconnect_blocker(local: XBeeDevice, remote: XBeeDevice, name: str):
    print(f'Link lost with {name}')
    reconnected = False
    while not reconnected:
        try:
            _ = local.read_data_from(remote)
        except XBeeException:
            time.sleep(1)
        else:
            reconnected = True  # Exception not raised, device reconnected!
    print(f'Link regained with {name}')


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
