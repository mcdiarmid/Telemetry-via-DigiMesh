# Telemetry-via-DigiMesh
Simple scripts that enable XBee radios with DigiMesh firmware to be used as telemetry radios for UAV operations

## License
This repository inherits two licenses from pre-requisite third-party libraries:

1. pySerial: BSD licence, (C) 2001-2017 Chris Liechti - 
https://github.com/pyserial/pyserial/blob/master/LICENSE.txt
2. python-xbee (Also dependant on pySerial): MPL 2.0 - https://github.com/digidotcom/python-xbee/blob/master/LICENSE.txt

## Linux Installation
Python3 - Arch based Linux Operating Systems:
'''bash
sudo pacman -S python python-pip
'''

Debian based Linux Operating Systems:
'''bash
sudo apt-get install python3 python3-pip
'''

Python3 libraries:
'''bash
sudo pip3 install digi-xbee pyserial
'''

XBee device rules installation:
(TODO)

[QGroundControl](https://github.com/mavlink/qgroundcontrol) has been the GCS of choice throughout 
development, but other versatile GCS such as [Mission 
Planner](https://github.com/ArduPilot/MissionPlanner) should be able to interface with these 
scripts easily.


## Windows 10
(Coming soon)
