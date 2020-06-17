fname = 'mavlink_messages_size_v'
endline = '])\n'


# RC Channels Override has variable length depending on controller
# 10 byte header, 2 uint8 fields, n_rc_channels uint16 fields, 2 crc bytes
n_rc_channels = 8

other_mav_sizes = {
    'ACTUATOR_CONTROL_TARGET': (
        10 + 41 + 2,
        'ID#140 Set the vehicle attitude and body angular rates.'
        ),
    'RC_CHANNELS_OVERRIDE': (
        10 + 2 + n_rc_channels*2 + 2,
        'ID#70 The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.'
        ),
    'COMMAND_LONG': (
        10 + 33 + 2,
        'ID#76 Send a command with up to seven parameters to the MAV. The command microservice is documented at https://mavlink.io/en/services/command.html'
        ),
    'ODOMETRY': (
        10 + 228 + 2,
        'ID#331 Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard for aerial vehicles (http://www.ros.org/reps/rep-0147.html).'
        ),
}

with open(f'{fname}1.py', 'r') as v1, open(f'{fname}2.py', 'w') as v2:
    # Iterate over v1 types and increase size by 4 to match additional header bytes
    while (line := v1.readline()):
        if line.startswith('(') and '#' in line:
            fields = line.split(',')
            new_val = int(fields[1].strip(' )')) + 4
            fields[1] = f'{new_val:>4})'
            line = ','.join(fields)
        elif line == endline:
            line = ''
        v2.write(line)

    # Iterate over v2 types that aren't defined in v1
    for mav_type in other_mav_sizes:
        mav_str = f"'{mav_type}'"
        mav_size, comment = other_mav_sizes[mav_type]
        v2.write(f'({mav_str:>42} ,{mav_size:>4}), # {comment}\n')
    
    v2.write(endline)
