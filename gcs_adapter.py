"""
XBee to UDP adapter script
Author: Campbell McDiarmid
"""


########################################################################################################################
#
#                                                    IMPORTS
#
########################################################################################################################


import logging
import os
import time
import argparse
import threading
import struct
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.exception import InvalidPacketException, TimeoutException, InvalidOperatingModeException, XBeeException
from commonlib import device_finder, setup_logging, MAVQueue, Fifo, XBEE_MAX_BAUD, XBEE_PKT_SIZE
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil


########################################################################################################################
#
#                                                  FUNCTIONS/CLASSES
#
########################################################################################################################


class UAVObject:
    """
    Class containing all of the useful queues, attributes, links, etc. that is associated with a UAV.
    """
    def __init__(self, name, ip, port, remote_device):
        """
        Initializer

        :param name: Name for this object
        :param ip: IP address for MAVLink UDP connection with GCS
        :param port: Port unique to this vehicle for MAVLink UDP connection with GCS
        :param remote_device: RemoteXBeeDevice object associated with this UAVs XBee radio
        """
        self.name = name
        self.ip = ip
        self.port = port
        self.queue_in = MAVQueue()
        self.queue_out = MAVQueue()
        self.socket = mavutil.mavudp(device=f'{ip}:{port}', input=False)
        self.parser = mavlink.MAVLink(Fifo())
        self.device = remote_device
        self.connected = True
        logging.info(f'Assigned {name} link to UDP {(ip, port)}')

    def __repr__(self):
        return f'"{self.name}"@{self.ip}:{self.port}'

    def __eq__(self, other):
        if isinstance(other, XBeeDevice) or isinstance(other, RemoteXBeeDevice):
            return self.device == other
        elif isinstance(other, UAVObject):
            return self == other
        else:
            raise TypeError(f'UAVObject only supports comparison to XBee objects or another UAVObject')


class XBee2UDP(object):
    def __init__(self, ip, serial_port, baud_rate, **kwargs):
        """
        Version 2 of the XBee to UDP class

        :param serial_port: Device location
        :param baud_rate: Baud rate
        :param udp: udp connection defaults (ip, port)
        :param kwargs: Other arguments passed to the XBee device initializer
        """
        # Initialize LUT for queues, MAVLink UDP sockets and stream parsers
        self.vehicles = []
        self.ip = ip

        # Initialize XBee device connection
        self.xbee = XBeeDevice(port=serial_port, baud_rate=baud_rate, **kwargs)
        while not self.xbee.is_open():
            try:
                self.xbee.open()
            except (InvalidPacketException, InvalidOperatingModeException, TimeoutException) as e:
                logging.exception(e)
                time.sleep(3)

        logging.info('XBee opened successfully.')
        self.main_running = False
        self.dev_running = {}
        self._udp_tx_closed = True

    def start(self):
        """
        Starts all processes and threads in appropriate order
        """
        self.main_running = True

        # Spawn UDP Tx thread
        _udp_tx_thread = threading.Thread(target=self._udp_tx_thread, daemon=True)
        _udp_tx_thread.start()

        # Once discovery has successfully completed, begin main loops
        _main_thread = threading.Thread(target=self._main_thread, daemon=True)
        _main_thread.start()

    def close(self):
        """
        Sets while-loop conditions to False for all threads, so each will run to completion and appropriate clean-up
        processes will be executed prior to the script terminating.
        """
        logging.info(f'Closing script...')
        self.main_running = False

        while self.vehicles:
            self.del_uav(self.vehicles[0])

    def del_uav(self, vehicle: UAVObject):
        """
        Deletes a UAVObject when a connection has been lost, or for whatever other reason a deletion is required

        :param vehicle: UAVObject unique to the vehicle in question
        """
        vehicle.connected = False
        time.sleep(0.02)
        self.vehicles.remove(vehicle)
        logging.info(f'Deleting connection with {vehicle}')
        del vehicle

    def new_uav(self, request_message):
        """
        Helper function for extracting useful data from a connection request, then initializing a UAVObject or the newly
        connected vehicle

        :param request_message: XBee packet containing information about the new UAV requesting a connection
        """
        # Extract information
        data = request_message.data
        identifier, port = struct.unpack('>BH', data)
        name = f'Navi {identifier}'
        device = request_message.remote_device

        # Create new vehicle object and acknowledge vehicle over radio
        vehicle = UAVObject(name, self.ip, port, device)
        self.vehicles.append(vehicle)
        self.xbee.send_data(device, b'COORD')

        # Start Rx UDP thread for this vehicle
        _udp_rx_thread_x = threading.Thread(target=self._udp_rx_thread, args=(vehicle,), daemon=True)
        _udp_rx_thread_x.start()

    def _main_thread(self):
        """
        Primary loop for servicing and passing data to the correct place:
        I.   Read messages from the XBee's Rx FIFO buffer, parse and place the in the appropriate queue.  New messages
             from undiscovered vehicles will trigger a process creating a UAVObject for the new vehicle.
        II.  Iterate through known devices, clear outgoing queue for each and attempt to transmit.  If there is an error
             in transmission this device will be deleted, along with all associated queues and UDP connections.
        III. TODO Check for a handover command.
        IV.  TODO Send broadcast to all UAVs if a hand-over is to occur.
        """

        while self.main_running:
            # I. Service messages from vehicles (incoming/Rx)
            rx_packet = self.xbee.read_data()
            # will replace with a walrus statement in python 3.8 (rx_packet := self.xbee.read_data())
            while rx_packet:
                # Check whether transmitting device is known or not
                if any([v == rx_packet.remote_device for v in self.vehicles]):
                    index = self.vehicles.index(rx_packet.remote_device)
                    vehicle = self.vehicles[index]
                    
                    try:
                        mav_msgs = vehicle.parser.parse_buffer(rx_packet.data)
                    except mavlink.MAVError as e:  # Check MAVLink message for errors
                        logging.exception(e)
                    else:
                        if mav_msgs:
                            vehicle.queue_in.extend(mav_msgs)
                else:
                    self.new_uav(rx_packet)  # Unknown transmitter - reply to connection request
                    time.sleep(0.01)
                    self.xbee.flush_queues()  # XBee queue may contain multiple connection requests from this device
                    time.sleep(0.01)

                rx_packet = self.xbee.read_data()

            # II. Service queues from GCS (outgoing/Tx)
            for vehicle in self.vehicles:
                outgoing = vehicle.queue_out.read_bytes()
                loop_time = XBEE_PKT_SIZE * 8 / XBEE_MAX_BAUD
                try:
                    while outgoing:
                        prev = time.time()
                        self.xbee.send_data(vehicle.device, outgoing[:XBEE_PKT_SIZE])
                        outgoing = outgoing[XBEE_PKT_SIZE:]
                        wait = loop_time + prev - time.time()
                        time.sleep(wait if wait > 0 else 0)
                except XBeeException as e:
                    logging.exception(e)
                    self.del_uav(vehicle)
                    break

            time.sleep(0.001)

        self.xbee.close()

    @staticmethod
    def _udp_rx_thread(vehicle: UAVObject):
        """
        One thread is created for each UDP-XBee connection required, as socket.recvfrom() is a blocking function, and
        will wait until data has been received from QGroundControl (or other GCS software).  By doing this, new data
        that is forwarded on from the XBee to QGroundControl is not held up by the recvfrom function.

        :param vehicle: UAVObject representing a connected vehicle
        """
        logging.info(f'Started UDP Rx thread for {vehicle}')
        while vehicle.connected:
            msg = vehicle.socket.recv_msg()
            if msg:
                vehicle.queue_out.write(msg)
            time.sleep(0.01)

    def _udp_tx_thread(self):
        """
        UDP transmission thread that iterates through the queues of each remote XBee device by Node ID and sends any
        data directly along the associated UDP socket to QGroundControl or other GCS.
        """
        logging.info(f'Started UDP Tx thread')
        while self.main_running:
            for vehicle in self.vehicles:
                if vehicle.queue_in:
                    msg = vehicle.queue_in.read()
                    vehicle.socket.write(msg.get_msgbuf())
            time.sleep(0.01)


def main(ip, baud_rate):
    """
    Main function and loop

    :param ip: IP address for MAVLink UDP connection with GCS
    :param baud_rate: Baud rate (bits per second) for serial communications with XBee radio.
    """
    xb_port = device_finder('XBee')
    software_adapter = XBee2UDP(ip, xb_port, baud_rate)
    software_adapter.start()

    try:
        while True:
            # TODO prompt here for duplicate UDP ports, Relay UAV handovers and other future additions
            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit) as e:
        logging.exception(e)
        logging.info('Closing Script')
        software_adapter.close()
        del software_adapter


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--baud', type=str, required=False, default=XBEE_MAX_BAUD,
        help='Baud rate (bits per second) for serial communications with XBee radio.')
    parser.add_argument(
        '--ip', type=str, required=False,
        help='IP address for UDP links back to GCS software')
    args = parser.parse_args()
    if args.ip:
        _ip = args.ip
    else:
        _ip, *_ = os.environ['SSH_CONNECTION'].split(' ')
    setup_logging('logs/gcs_logging.json')
    main(_ip, args.baud)
