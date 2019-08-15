"""
PX4 to XBee serial adapter script
Author: Campbell McDiarmid
"""

########################################################################################################################
#
#                                                    IMPORTS
#
########################################################################################################################


from digi.xbee.devices import XBeeDevice
import serial
import time
import threading
from pymavlink import mavutil
from commonlib import device_finder


########################################################################################################################
#
#                                                    CONSTANTS
#
########################################################################################################################


XBEE_PKT_MAX = 0xFF
SERIAL_BUFFER_MAX = 0xFFF
ARBITRARY_MIN = 100
XBEE_MAX_BAUD = 230400
PX4_COMPANION_BAUD = 921600
GCS_64BIT_ADDR = '0013a20040d68c2e'
PX4_MAV_PERIODS = {
	# Hard-coded Tx rates for specific MAVLink messages
	'ATTITUDE':            0.25,
	'VFR_HUD':             1.25,
	'HEARTBEAT':           2,
	'ATTITUDE_TARGET':     2.5,
	'HIGHRES_IMU':         3.25,
	'SYS_STATUS':          5,
	'BATTERY_STATUS':      5,
	'SERVO_OUTPUT_RAW':    5,
	'EXTENDED_SYS_STATE':  5,
	'ALTITUDE':            5,
	'LOCAL_POSITION_NED':  5,
	'ESTIMATOR_STATUS':    10,
	'VIBRATION':           10,
	'PING':                20,
}
MAV_SEQ_BYTE = 2

########################################################################################################################
#
#                                                  FUNCTIONS/CLASSES
#
########################################################################################################################


def obtain_network(xbee: XBeeDevice):
	# Discover network.  Repeat until GCS has been found.
	network = xbee.get_network()
	while True:
		print('Discovering network, GCS not found yet.')
		network.start_discovery_process()

		# Block until discovery is finished
		while network.is_discovery_running():
			time.sleep(0.1)

		# Check devices on the network by Node ID
		for device in network.get_devices():
			address = device.get_64bit_addr().address.hex()
			print(f'XBee device found with 64bit address: {address}')
			if address == GCS_64BIT_ADDR:
				return device, network


def mav_rx_thread(mav_device: mavutil.mavserial, sleep_time=0.0001):
	while True:
		m = mav_device.recv_msg()
		time.sleep(sleep_time)


def main():
	# TODO: Add command line argument passing
	# Find PX4 Device and open a serial connection
	px4_port = device_finder('PX4')
	# px4 = serial.Serial(px4_port, PX4_DEFAULT_BAUD)
	px4 = mavutil.mavserial(px4_port, PX4_COMPANION_BAUD)

	# Find XBee Device and open a serial connection
	xbee_port = device_finder('XBee')
	xb = XBeeDevice(xbee_port, XBEE_MAX_BAUD)
	xb.open()

	gcs, network = obtain_network(xb)
	next_times = {time.time() + PX4_MAV_PERIODS[k] for k in PX4_MAV_PERIODS}
	seq_counter = 0

	_parse_thread = threading.Thread(target=mav_rx_thread, args=(px4,), daemon=True)
	_parse_thread.start()

	# Infinite loop for bridging serial connection between PixHawk and XBee
	while True:
		# Check whether messages are scheduled for transmission
		data = b''
		for mavtype in PX4_MAV_PERIODS:
			if time.time() >= next_times[mavtype]:
				seq_counter = seq_counter + 1 if seq_counter < 254 else 0
				msgbytes = bytes(px4.messages[mavtype].get_msgbuf())
				msgbytes[MAV_SEQ_BYTE] = seq_counter
				radio_tx_data += msgbytes
				next_times[mavtype] += PX4_MAV_PERIODS[mavtype]

		# Break data up into packets of maximum length XBEE_PKT_MAX
		while data:
			pkt_data, data = data[:XBEE_PKT_MAX], data[XBEE_PKT_MAX:]
			pkt_sent = xb.send_data(gcs, pkt_data)

		# Read XBee, Write to PX4
		message = xb.read_data_from(gcs)
		if message:
			data = message.data
			try:
				print('Writing bytearray')
				px4.write(data)
			except Exception as e:
				print('Writing bytes')
				px4.write(bytes(data))

		time.sleep(0.001)


if __name__ == '__main__':
	main()
