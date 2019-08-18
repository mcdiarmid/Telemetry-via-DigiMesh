import os
import platform
import time
import serial.tools.list_ports as list_ports


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
