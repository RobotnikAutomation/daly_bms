# Daly BMS

ROS package that uses [python-daly-bms driver](https://github.com/dreadnought/python-daly-bms) data to read information from Daly BMS devices and publishes it using robotnik_msgs/BatteryStatus message type.

## Installation

```
sudo apt install python3-rospkg-modules
```

Install [python-daly-bms driver](https://github.com/dreadnought/python-daly-bms) by following the instructions provided in the README.md file. This is a python library independent from ROS, so there is no need to install it in the catkin workspace.

```
git clone https://github.com/RobotnikAutomation/daly_bms.git
```

```
catkin build daly_bms
```

### Dependencies

For *serial* connections:
```
pip3 install pyserial
```

## Parameters

- **port** (String, /dev/ttyUSB_BMS): Serial port of the device

## Additional information

This package includes an udev rule to find the USB device using its model and vendor identifiers. This is useful to bind the device to a specific port, independently of the actual physical port where the device is plugged. To add this udev rule to your system, just copy it into your /etc/udev/rules.d directory.
