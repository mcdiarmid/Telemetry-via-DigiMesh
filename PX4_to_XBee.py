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
import time
import threading
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from commonlib import device_finder, MAVQueue, Fifo, replace_seq


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
    'ATTITUDE':            		0.125,
    'VFR_HUD':             		0.625,
    'ODOMETRY':					0.825,
    'COMMAND_LONG':	       		1,
    'ATTITUDE_QUATERNION': 		1,
    'ACTUATOR_CONTROL_TARGET': 	1,
    'TIMESYNC':	       			1,
    'SYSTEM_TIME':         		1,
    'HEARTBEAT':           		1,
    'ATTITUDE_TARGET':     		1.25,
    'HIGHRES_IMU':         		1.67,
    'SYS_STATUS':          		2.5,
    'BATTERY_STATUS':      		2.5,
    'SERVO_OUTPUT_RAW':    		2.5,
    'EXTENDED_SYS_STATE':  		2.5,
    'ALTITUDE':            		2.5,
    'LOCAL_POSITION_NED':  		2.5,
    'ESTIMATOR_STATUS':    		5,
    'VIBRATION':           		5,
    'PING':                		10,
}
MAV_IGNORES = ['BAD_DATA']
MAV_SEQ_BYTE = 4

########################################################################################################################
#
#                                                  FUNCTIONS/CLASSES
#
########################################################################################################################


def obtain_network(xbee: XBeeDevice):
    """

    :param xbee: Local XBee device object
    :return: Remote XBee device object and DigiMesh network object
    """
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


def mav_rx_thread(mav_device: mavutil.mavserial, priority_queue: MAVQueue, sleep_time=0.0005):
    """
    This function serves the purpose of receiving messages from the flight controller at such a rate that no buffer
    overflow occurs.  When mav_device.recv_msg() is called, if enough data has come in from the serial connection to
    form a MAVLink packet, the packet is parsed and the latest copy of the particular type of MAVLink message is updated
    in the mav_device.messages dict.  This dict is used in the main thread for scheduling the transmission of each type
    of MAVLink packet, effectively decimating the stream to a set rate for each type of MAVLink message.

    :param mav_device: MAVLink serial connection object
    :param priority_queue: Queue for messages that come through as a result of a GCS request
    :param sleep_time: Sleep time between loops
    """
    print(f'Started MAVLink Rx Thread')
    while True:  # Loop forever
        m = mav_device.recv_msg()
        if m:
            if m.get_type() not in PX4_MAV_PERIODS and m.get_type() not in MAV_IGNORES:
                priority_queue.write(m)
        time.sleep(sleep_time)


if __name__ == '__main__':
    # TODO: Add command line argument passing + big tidy
    # Find PX4 Device and open a serial connection
    px4_port = device_finder('FT232R USB UART')  # TODO Will need to make a rules file to identify PX4 via FTDI @ TELEM2
    px4 = mavutil.mavserial(px4_port, PX4_COMPANION_BAUD)

    # Find XBee Device and open a serial connection
    xbee_port = device_finder('XBee')
    xb = XBeeDevice(xbee_port, XBEE_MAX_BAUD)
    xb.open()

    # Get Remote XBee object for GCS transceiver and DigiMesh Network object
    gcs, network = obtain_network(xb)

    # Generate a dictionary for keeping track of when each message is scheduled to be sent next
    next_times = {k: time.time() + PX4_MAV_PERIODS[k] for k in PX4_MAV_PERIODS}
    seq_counter = 0

    # Priority Queue for servicing GCS requests
    priority_queue = MAVQueue()
    parser = mavlink.MAVLink(Fifo())

    # Separate thread for constantly receiving and parsing new MAVLink packets from the flight controller
    _parse_thread = threading.Thread(target=mav_rx_thread, args=(px4, priority_queue,), daemon=True)
    _parse_thread.start()

    # Infinite loop for bridging serial connection between PixHawk and XBee
    print(f'Started main message handling loop')
    while True:
        # Reset Tx buffer
        tx_buffer = b''

        # Add any prioritized messages to front of Tx buffer
        while priority_queue:
            msg = priority_queue.read()
            msg_bytes = replace_seq(msg, seq_counter)
            tx_buffer += msg_bytes
            seq_counter += 1
            print(f'Priority message of type: {msg.get_type()}')

        # Add any prioritized messages to the end of the Tx buffer
        for mav_type in PX4_MAV_PERIODS:
            if time.time() >= next_times[mav_type]:
                next_times[mav_type] = time.time() + PX4_MAV_PERIODS[mav_type]

                if mav_type not in px4.messages:
                    print(f'MAVLink message of type {mav_type} has not yet been received!')
                    continue
                msg = px4.messages[mav_type]
                msg_bytes = replace_seq(msg, seq_counter)
                tx_buffer += msg_bytes
                seq_counter += 1

        # Break data up into packets of maximum length XBEE_PKT_MAX
        while tx_buffer:
            pkt_data, tx_buffer = tx_buffer[:XBEE_PKT_MAX], tx_buffer[XBEE_PKT_MAX:]
            pkt_sent = xb.send_data(gcs, pkt_data)
            try:
                parser.parse_buffer(pkt_data)
            except mavlink.MAVError as e:
                print(e)
        # Read XBee, Write to PX4
        message = xb.read_data_from(gcs)
        if message:
            data = message.data
            try:
                gcs_msg = px4.mav.decode(data)
                msg_type = gcs_msg.get_type()
                print(f'GCS message of type: {msg_type}')
            except mavlink.MAVError as e:
                print(e)
                continue

            if msg_type == 'HEARTBEAT':
                """
                HEARTBEAT reply/"acknowledgement"
                Need to manually construct a RADIO_STATUS MAVLink message and place it at the front of 
                priority_queue, as RADIO_STATUS messages are automatically constructed and sent back to the 
                GCS on SiK radio firmware in response to a HEARTBEAT.  This is crucial for establishing a
                recognisable link on GCS software, such as QGroundControl.
                """
                # TODO: RSSI, Remote RSSI, TxBuf, RxErrors and fixed should be obtainable - currently filler values
                radio_status_msg = px4.mav.radio_status_encode(
                    rssi=210,
                    remrssi=215,
                    txbuf=100,
                    noise=51,
                    remnoise=41,
                    rxerrors=0,
                    fixed=0)
                radio_status_msg.pack(px4.mav)
                priority_queue.write(radio_status_msg)
            elif msg_type == 'BAD_DATA':
                continue

            px4.write(data)

        time.sleep(0.001)
