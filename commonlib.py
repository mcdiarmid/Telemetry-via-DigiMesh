import os
import platform
import time
import serial.tools.list_ports as list_ports
from pymavlink.mavutil import x25crc

MAVLINK_SEQ_BYTE = 4
MAVLINK_MESSAGE_CRCS = (
    50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0,
    214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244,
    242, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 214, 223,
    141, 33, 15, 3, 100, 24, 239, 238, 30, 240, 183, 130, 130, 0, 148, 21, 0,
    243, 124, 0, 0, 0, 20, 0, 152, 143, 0, 0, 127, 106, 0, 0, 0, 0, 0, 0, 0,
    231, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 0, 0, 0,
    0, 235, 93, 124, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 241, 15, 134, 219, 208,
    188, 84, 22, 19, 21, 134, 0, 78, 68, 189, 127, 111, 21, 21, 144, 1, 234,
    73, 181, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 204, 49, 170, 44, 83, 46, 0)


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


def print_msg(name, start, data, is_incoming=True):
    # Formats incoming and outgoing messages to fit the terminal window
    cols, rows = os.get_terminal_size()
    elapsed = time.time() - start
    lhs = 6  # Left hand side width in monospace unicode characters

    while elapsed >= 10:  # Increase LHS for each digit of elapsed time
        elapsed %= 10
        lhs += 1

    rhs = cols - lhs - 2  # Right hand side width
    bpl = rhs // 4  # Bytes displayed per line
    brkline = '-' * cols + '\n'  # Line of hyphens
    line_format = '{' + f': <{lhs}' + '}| {' + f': <{rhs}' + '}\n'
    txt = brkline + line_format.format(f'{time.time() - start:.2f}s', name)
    sep = ', '

    direction = 'IN' if is_incoming else 'OUT'

    nlines, rem = divmod(len(data), bpl)
    nlines += rem > 0
    for i in range(nlines):
        linedata = sep.join('%02X' % c for c in data[i * bpl:(i + 1) * bpl])
        line = line_format.format(f'{direction}[{i}]', linedata)
        txt += line
    if nlines:
        txt += brkline

    print(txt)


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
    cc.accumulate(MAVLINK_MESSAGE_CRCS[msg.get_msgId()])
    data[-2], data[-1] = cc.crc & 0xFF, cc.crc >> 8
    return bytes(data)
