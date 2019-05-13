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
PX4_DEFAULT_BAUD = 115200


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
			print('XBee device found with Node ID: {}'.format(device.get_node_id()))
			if device.get_node_id() == 'GCS':
				return device, network


def main():
	# TODO: Add command line argument passing
	# Find PX4 Device and open a serial connection
	px4_port = device_finder('PX4')
	px4 = serial.Serial(px4_port, PX4_DEFAULT_BAUD)

	# Find XBee Device and open a serial connection
	xbee_port = device_finder('XBee')
	xb = XBeeDevice(xbee_port, XBEE_MAX_BAUD)
	xb.open()

	gcs, network = obtain_network(xb)

	# Infinite loop for bridging serial connection between PixHawk and XBee
	while True:
		# Read PX4, Write to XBee
		if ARBITRARY_MIN <= px4.in_waiting < SERIAL_BUFFER_MAX:
			data = px4.read(px4.in_waiting)

			# Break data up into packets of maximum length XBEE_PKT_MAX
			while data:
				pkt_data, data[:] = data[:XBEE_PKT_MAX], data[XBEE_PKT_MAX:]
				pkt_sent = xb.send_data(gcs, pkt_data)

		# Reset input buffer if overflow has occurred
		elif px4.in_waiting == SERIAL_BUFFER_MAX:
			px4.reset_input_buffer()

		# Read XBee, Write to PX4
		message = xb.read_data_from(gcs)
		if message:
			data = bytes(message.data)
			px4.write(data)

		time.sleep(0.001)


if __name__ == '__main__':
	main()
