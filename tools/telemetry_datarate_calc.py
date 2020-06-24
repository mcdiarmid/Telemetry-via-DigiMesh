from mavlink_messages_size_v2 import mavlink_message_lengths_dict
import json

uav_settings_json = open('../uav_settings.json', 'r')
msg_periods = json.load(uav_settings_json)['mav_rates']
ignore = [
    'RC_CHANNELS_OVERRIDE', 'RC_CHANNELS',
    'ODOMETRY',
##    'SERVO_OUTPUT_RAW',
##    'GPS_RAW_INT',
##    'LOCAL_POSITION_NED',
]

data_rate = 0

# Iterates over all MAV types defined in uav_settings.json to calculate the average telemetry rate (Upper bound)
for i, msg_type in enumerate(sorted(msg_periods.keys(), key=len)):
    if msg_type in ignore:
        continue

    size_bits = 8*mavlink_message_lengths_dict[msg_type]
    period = msg_periods[msg_type]

    if msg_type == 'HEARTBEAT':  # Since GCS also sends a HEARTBEAT to each UAV
        size_bits += 8*(mavlink_message_lengths_dict[msg_type]+mavlink_message_lengths_dict['RADIO_STATUS'])

    data_rate += size_bits/period


print(int(data_rate))
