# Telemetry-via-DigiMesh
Simple scripts that enable XBee radios with DigiMesh firmware to be used as telemetry radios for UAV operations

## License
This repository inherits two licenses from pre-requisite third-party libraries:

1. pySerial: BSD licence, (C) 2001-2017 Chris Liechti - 
https://github.com/pyserial/pyserial/blob/master/LICENSE.txt
2. python-xbee (Also dependant on pySerial): MPL 2.0 - https://github.com/digidotcom/python-xbee/blob/master/LICENSE.txt

## Linux Installation
Python3 - Arch based Linux Operating Systems:

```bash
sudo pacman -S python python-pip
```

Debian based Linux Operating Systems:

```bash
sudo apt-get install python3 python3-pip
```

Python3 libraries:

```bash
sudo pip3 install digi-xbee pyserial
```

XBee device rules installation:

(TODO)

[QGroundControl](https://github.com/mavlink/qgroundcontrol) has been the GCS software of choice 
throughout 
development, but other versatile GCS software such as [Mission 
Planner](https://github.com/ArduPilot/MissionPlanner) should be able to interface with these 
scripts easily.

## Windows 10
(Coming soon)

## Usage
The purpose of these scripts are to enable the use of XBee DigiMesh without the need to 
modify the Pixhawk PX4 firmware, or compiling a custom build for GCS software.  

* ```PX4_to_XBee.py``` has been designed to operate on a companion computer that is connected to 
both a Pixhawk and an XBee via USB.
* ```XBee_to_UDP.py``` acts as an interface between an XBee device and GCS software.  For each 
XBee device connected to the mesh network, a UDP socket is created and connected to a user 
defined port (UDP server hosted by GCS software).

## TODO

Currently, these scripts have one purpose - to create multiple one-to-one GCS to vehicle MAVLink 
connections that share the same wireless DigiMesh network and one common GCS XBee radio.  However, 
future commits will aim to improve the usefulness, versatility and reliability of these scripts.  
Below is a prioritized list of TODOs as of April 1<sup>st</sup> 2019:

1. Prevent threads from crashing when UDP servers are closed.  Attempt to reconnect.
2. USB rules for GCS computer and companion computer so admin priviledges are not required to run 
scripts.
3. Remove magic numbers and add command line argument passing.  Overall code tidy-up.
4. Exit scripts appropriately by closing all open ports and serial connections.
5. Cleaner initialization process for all GCS and vehicles on the mesh network.
6. Live console for functions to be executed during script operation.
7. Investigate rediscovery of devices that have left the mesh network and rejoined.
8. Non-explicit broadcasts to all devices on the network.
9. Ensure threads that may access the same area of memory cannot do so simultaneously, 
avoiding unnecessary crashes.
10. Interface option for companion computer script to interface with other companion computer 
software, as opposed to directly with the Pixhawk.
11. Windows support for GCS script.
12. TCP and serial options for interfacing with GCS software.
