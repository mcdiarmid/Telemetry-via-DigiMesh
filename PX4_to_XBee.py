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
from digi.xbee.exception import XBeeException
import time
import threading
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from commonlib import device_finder, MAVQueue, Fifo, replace_seq, \
    reconnect_blocker, send_buffer_limit_rate, mav_rx_thread, PX4_MAV_PERIODS


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
MAV_IGNORES = ['BAD_DATA']


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
    def callback(remote):
        print(remote)
    network.add_device_discovered_callback(callback)

    while True:
        print('Discovering network, GCS not found yet.')
        network.start_discovery_process()

        # Block until discovery is finished
        while network.is_discovery_running():
            time.sleep(0.1)

        print(network.get_devices())
        # Check devices on the network by Node ID
        for device in network.get_devices():
            address = device.get_64bit_addr().address.hex()
            print(f'XBee device found with 64bit address: {address}')
            if address == GCS_64BIT_ADDR:
                return device, network


def main(relay=False):
    # TODO: Add command line argument passing + big tidy
    # Find PX4 Device and open a serial connection
    px4_port = device_finder('FT232R USB UART')  # TODO Will need to make a rules file to identify PX4 via FTDI @ TELEM2
    px4 = mavutil.mavserial(px4_port, PX4_COMPANION_BAUD, source_system=19, source_component=1)

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
        try:
            send_buffer_limit_rate(xb, gcs, tx_buffer, 28800)
        except XBeeException:
            reconnect_blocker(xb, gcs, 'GCS')
            priority_queue = MAVQueue()  # Reset priority queue once reconnected
            continue

        # Read XBee, Write to PX4
        try:
            message = xb.read_data_from(gcs)
        except XBeeException:
            reconnect_blocker(xb, gcs, 'GCS')
            priority_queue = MAVQueue()  # Reset priority queue once reconnected
            continue

        if message:
            data = message.data
            try:
                gcs_msg = px4.mav.decode(data)
                msg_type = gcs_msg.get_type()
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


if __name__ == '__main__':
    main()
