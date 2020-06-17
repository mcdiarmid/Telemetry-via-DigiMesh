from mavlink_messages_size_v2 import mavlink_message_lengths_dict
import json

uav_settings_json = open('../uav_settings.json', 'r')
msg_periods = json.load(uav_settings_json)['mav_rates']
ignore = ['RC_CHANNELS_OVERRIDE', 'RC_CHANNELS']

data_rate = 0

# Iterates over all MAV types defined in uav_settings.json to calculate the average telemetry rate (Upper bound)
for msg_type in msg_periods:
    if msg_type in ignore:
        continue
    data_rate += 8*mavlink_message_lengths_dict[msg_type]/msg_periods[msg_type]

print(int(data_rate))
