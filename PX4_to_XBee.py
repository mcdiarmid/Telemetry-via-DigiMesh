from digi.xbee.devices import XBeeDevice
import serial, time

def main():
	px4 = serial.Serial('/dev/ttyACM0', 115200)
	xb = XBeeDevice('/dev/ttyUSB0', 230400)
	xb.open()
	
	network = xb.get_network()
	network.start_discovery_process()
	
	while network.is_discovery_running():
		time.sleep(0.1)
	
	GCS = None
	
	for device in network.get_devices():
		if device.get_node_id() == 'GCS':
			GCS = device
			
	if GCS is None:
		raise Exception('Ground Control Station not discovered.')
	
	while True:
		# Read PX4, Write to XBee
		if 100 <= px4.in_waiting < 4095:
			data = px4.read(px4.in_waiting)
			while data:
				print('[PX4 -> XBee] {}'.format(data[:255]))
				pkt_sent = xb.send_data(GCS, data[:255])
				data = data[255:]
		elif px4.in_waiting == 4095:
			px4.reset_input_buffer()
		else:
			pass

		# Read XBee, Write to PX4
		message = xb.read_data_from(GCS)
		if message:
			data = bytes(message.data)
			print('[XBee -> PX4] {}'.format(data))
			px4.write(data)
		
		time.sleep(0.001)

if __name__ == '__main__':
	main()

