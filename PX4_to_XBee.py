from digi.xbee.devices import XBeeDevice
import serial, time
import serial.tools.list_ports as list_ports

# Constants used throughout script
XBEE_PKT_MAX = 0xFF
SERIAL_BUFFER_MAX = 0xFFF
ARBITRARY_MIN = 100
XBEE_MAX_BAUD = 230400
PX4_DEFAULT_BAUD = 115200


def device_finder(name):
	devport = None
	while devport is None:
		for dev in list_ports.comports():
			if name in dev.product:
				devport = dev.device
		if devport:
			print('{} found'.format(name))
		else:
			print('Please insert {} device'.format(name))
			time.sleep(5)
	return devport


def main():
	# TODO: Add command line argument passing
	# Find PX4 Device and open a serial connection
	px4port = device_finder('PX4')
	px4 = serial.Serial(px4port, PX4_DEFAULT_BAUD)

	# Find XBee Device and open a serial connection
	xbeeport = device_finder('XBee')
	xb = XBeeDevice(xbeeport, XBEE_MAX_BAUD)
	xb.open()

	# Discover network.  Repeat until GCS has been found.
	GCS = None
	network = xb.get_network()
	while GCS is None:
		print('Discovering network, GCS not found yet.')
		network.start_discovery_process()
		while network.is_discovery_running():
			time.sleep(0.1)
		for device in network.get_devices():
			print('UAV found with Node ID: {}'.format(device.get_node_id()))
			if device.get_node_id() == 'GCS':
				GCS = device

	# Infinite loop for bridging serial connection between Pixhawk and XBee
	while True:
		# Read PX4, Write to XBee
		if ARBITRARY_MIN <= px4.in_waiting < SERIAL_BUFFER_MAX:
			data = px4.read(px4.in_waiting)
			# Break data up into packets of maximum length XBEE_PKT_MAX
			while data:
				print('[PX4 -> XBee] {}'.format(data[:XBEE_PKT_MAX]))
				pkt_sent = xb.send_data(GCS, data[:XBEE_PKT_MAX])
				data = data[XBEE_PKT_MAX:]
		elif px4.in_waiting == SERIAL_BUFFER_MAX:
			# Reset input buffer if overflow has occured
			px4.reset_input_buffer()

		# Read XBee, Write to PX4
		message = xb.read_data_from(GCS)
		if message:
			# TODO: is there a maximum input size?
			data = bytes(message.data)
			print('[XBee -> PX4] {}'.format(data))
			px4.write(data)

		time.sleep(0.001)

if __name__ == '__main__':
	main()
