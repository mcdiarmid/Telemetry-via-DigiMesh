from digi.xbee.devices import XBeeDevice
import serial
import time
from commonlib import device_finder


# Constants used throughout script
XBEE_PKT_MAX = 0xFF
SERIAL_BUFFER_MAX = 0xFFF
ARBITRARY_MIN = 100
XBEE_MAX_BAUD = 230400
PX4_DEFAULT_BAUD = 115200


def main():
	# TODO: Add command line argument passing
	# Find PX4 Device and open a serial connection
	px4_port = device_finder('PX4')
	px4 = serial.Serial(px4_port, PX4_DEFAULT_BAUD)

	# Find XBee Device and open a serial connection
	xbee_port = device_finder('XBee')
	xb = XBeeDevice(xbee_port, XBEE_MAX_BAUD)
	xb.open()

	# Discover network.  Repeat until GCS has been found.
	ground_control_station = None
	network = xb.get_network()
	while ground_control_station is None:
		print('Discovering network, GCS not found yet.')
		network.start_discovery_process()
		while network.is_discovery_running():
			time.sleep(0.1)
		for device in network.get_devices():
			print('UAV found with Node ID: {}'.format(device.get_node_id()))
			if device.get_node_id() == 'GCS':
				ground_control_station = device

	# Infinite loop for bridging serial connection between PixHawk and XBee
	while True:
		# Read PX4, Write to XBee
		if ARBITRARY_MIN <= px4.in_waiting < SERIAL_BUFFER_MAX:
			data = px4.read(px4.in_waiting)
			# Break data up into packets of maximum length XBEE_PKT_MAX
			while data:
				print('[PX4 -> XBee] {}'.format(data[:XBEE_PKT_MAX]))
				pkt_sent = xb.send_data(ground_control_station, data[:XBEE_PKT_MAX])
				data = data[XBEE_PKT_MAX:]
		elif px4.in_waiting == SERIAL_BUFFER_MAX:
			# Reset input buffer if overflow has occurred
			px4.reset_input_buffer()

		# Read XBee, Write to PX4
		message = xb.read_data_from(ground_control_station)
		if message:
			# TODO: is there a maximum input size?
			data = bytes(message.data)
			print('[XBee -> PX4] {}'.format(data))
			px4.write(data)

		time.sleep(0.001)


if __name__ == '__main__':
	main()
