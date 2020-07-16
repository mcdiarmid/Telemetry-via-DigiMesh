# Telemetry-via-DigiMesh
Digi XBee Pro 868/900 radio modules offer similar hardware specifications to most of the telemetry radios that are supported by PX4.  However, XBee's can be loaded with a mesh networking firmware called DigiMesh that allows for multiple units to be connected to one network, where each can be configured as a router, relay or endpoint.  Ready-to-go mesh networking, low cost, 200kbps Tx/Rx and industry, scientific and medical (ISM) radio band operation make XBee radios ideal for prototying Swarm unmanned aerial vehicle (UAV) systems with low over-the-air data-rate requirements.  Despite this, the PX4's PixHawk Firmware and ground control station (GCS) software have both dropped direct interface support for the XBee radio.  Therefore, this repository was spawned to provide a simple and (mostly) hands-off software adapter to bridge XBee radio hardware with both the PixHawk and QGroundControl, without altering the software for either.  This allows one GCS to control multiple UAVs over a singular mesh network. 


The repository was created to support a Master of Engineering (ME) project, where one GCS will be used to control multiple UAVs.  One or more UAVs will be flying within line of sight (LOS) at a moderate to high altitude with the purpose of acting as an airbourne communications relay for one or more low-altitude UAVs, all flying beyond the visual line of sight (BVLOS) and tasked with data collection.

## License
This repository inherits two licenses from pre-requisite third-party libraries:

1. pySerial: BSD licence, (C) 2001-2017 Chris Liechti - 
https://github.com/pyserial/pyserial/blob/master/LICENSE.txt
2. python-xbee (Also dependant on pySerial): MPL 2.0 - https://github.com/digidotcom/python-xbee/blob/master/LICENSE.txt

## Linux Installation
Python3 - Arch based Linux Operating Systems (Python 3.7 is installed by default on fresh Arch based install):

```bash
$ sudo pacman -S python python-pip
```

Debian based Linux Operating Systems:

```bash
$ sudo apt-get install python3 python3-pip
```

Python3 libraries:

```bash
$ sudo pip3 install digi-xbee pyserial pymavlink
```

Device rules installation (requires root):

```bash
# cp devices/rules/*.rules /etc/udev/rules.d/
# udevadm control --reload && udevadm trigger
```

[QGroundControl](https://github.com/mavlink/qgroundcontrol) has been the GCS software of choice 
throughout development, but other versatile GCS software such as [Mission Planner](https://github.com/ArduPilot/MissionPlanner)
should be able to interface with these scripts easily.

## Usage
The purpose of these scripts are to enable the use of XBee DigiMesh without the need to 
modify the Pixhawk PX4 firmware, or compiling a custom build for GCS software.  

* ```px4.py``` has been designed to operate on a companion computer that is connected to
both a Pixhawk and either an XBee radio or WiFi module.  If using an XBee, a link negotiation is performed with
the coordinator computer running ```gcs.py```.  If using WiFi with the ```--ssh``` flag, this script is 
intended to communicated directly to QGroundControl over UDP.

* By default ```px4.py``` loads  ```uav_settings.json``` for parameters and settings related to the UAV, along with
a look-up-table that defines the period between transmissions for each routinely scheduled MAVLink message.  This was added
to effectively decimate the rate of data coming out of the Pixhawk to very easily meet any bandwidth constraints.  Other 
parameters include MAVLink version, UDP port and vehicle ID.

* ```gcs.py``` acts as an interface between an XBee device and GCS software.  For each
XBee device connected to the mesh network, a UDP socket is created and connected to a user 
defined port (UDP server hosted by GCS software).

Dedicated endpoint or relay in a purely DigiMesh network:
```bash
nohup python3 px4.py &
```

GCS computer running QGroundControl using DigiMesh to communicate to endpoint UAVs:
```bash
python3 gcs.py --ip 127.0.0.1
```

Relay UAV communicating to endpoint UAVs via DigiMesh and WiFi to bridge connections back to the GCS computer:
```bash
nohup python3 px4.py --ssh &
nohup python3 gcs.py &
```

*Note: The nohup should be used if you intend to start the script over SSH and then close the session once everything has started correctly* 
