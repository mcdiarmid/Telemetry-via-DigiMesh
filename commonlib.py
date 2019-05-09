import os
import time
import serial.tools.list_ports as list_ports


def device_finder(name):
    dev_port = None
    while dev_port is None:
        for comport in list_ports.comports():
            if name in comport.product:
                dev_port = comport.device
        if dev_port:
            print('{} found'.format(name))
        else:
            print('List of Comports: {}'.format(list_ports.comports()))
            print('Please insert {} device'.format(name))
            time.sleep(5)
    return dev_port


def print_msg(key, start, incoming=b'', outgoing=b''):
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
    txt = brkline + line_format.format(f'{time.time() - start:.2f}s', key)
    sep = ', '

    for direction, data in (('IN', incoming), ('OUT', outgoing)):
        nlines, rem = divmod(len(data), bpl)
        nlines += rem > 0
        for i in range(nlines):
            linedata = sep.join('%02X' % c for c in data[i * bpl:(i + 1) * bpl])
            line = line_format.format(f'{direction}[{i}]', linedata)
            txt += line
        if nlines:
            txt += brkline

    print(txt)
