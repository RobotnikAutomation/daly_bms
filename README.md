# Daly Battery Driver

ROS package that uses [python-daly-bms driver](https://github.com/dreadnought/python-daly-bms) data to read information from Daly BMS devices and publishes it using robotnik_msgs/BatteryStatus message type.

## Installation

```bash
command -v vcs >/dev/null 2>&1 || (sudo apt update && sudo apt install -y python3-vcstool)
command -v pip3 >/dev/null 2>&1 || (sudo apt update && sudo apt install -y python3-pip)
mkdir -p robot_ws/src
cd robot_ws/src
git clone -b ros2-devel git@github.com:RobotnikAutomation/daly_bms.git
vcs import \
    --input daly_bms/common.repos.yaml \
    --shallow \
    .
pip3 install -r daly_bms/requirements.txt
cd ..
rosdep install
cd src/daly_bms
colcon build
```

## Run
```bash
source install/setup.bash
ros2 launch daly_bms daly_bms.launch.xml
```
### Launch File Arguments

The launch file accepts the following arguments:

1. **port**
   - Description: Specifies the serial port for the BMS connection.
   - Default value: Set by the environment variable `ROBOT_BMS_PORT`, or `/dev/ttyUSB_BMS` if not set.
   - Usage: `ros2 launch daly_bms daly_bms.launch.xml port:=/dev/ttyUSB0`

2. **node_name**
   - Description: Sets the name of the ROS2 node.
   - Default value: Set by the environment variable `NODE_NAME`, or `battery_estimator` if not set.
   - Usage: `ros2 launch daly_bms daly_bms.launch.xml node_name:=my_bms_node`

3. **robot_id**
   - Description: Defines the robot identifier, used as the namespace for the node.
   - Default value: Set by the environment variable `ROBOT_ID`, or `robot` if not set.
   - Usage: `ros2 launch daly_bms daly_bms.launch.xml robot_id:=my_robot`

4. **log_level**
   - Description: Sets the logging level for the node.
   - Default value: Set by the environment variable `LOG_LEVEL`, or `INFO` if not set.
   - Usage: `ros2 launch daly_bms daly_bms.launch.xml log_level:=DEBUG`


### Ros run parameters

- **serial_port** (String, `/dev/ttyUSB_BMS`): Serial port of the device

## Containers

### Setup the containers
The container setup is controlled by the [`setup-container.yaml`](./setup-container.yaml) file. This file contains essential variables for creating the required Docker image files. Review and edit this file as needed before proceeding with the setup.

```yaml
---
images:
  version: devel
  sites:
    local: docker-hub
    # local: robotnik

ros:
  general:
    distro: humble
    domain_id: 50
    robot_id: robot
    log_level: INFO
  bms:
    port: /dev/ttyUSB_BMS
```

#### Key Configuration Parameters

- `images.version`: Specifies the version tag for the Docker image.
- `images.sites`: Defines the Docker registry locations for local environment. (Options are `docker-hub` or `robotnik`)
- `ros.general`: Contains ROS 2 specific configurations, including the distribution and robot settings.
- `ros.bms`: Specifies hardware-related settings, such as the BMS port.

#### Creating Container Files

To create the container files use the following command
```bash
setup-container/scripts/setup.sh
```
This script will process the `setup-container.yaml` file and create all required Docker-related files in the appropriate directories.

### Create Container
To build the container, navigate to the builder directory and use Docker Compose. This step compiles the necessary images based on the Dockerfile provided in the directory.
```bash
cd container/builder
docker compose build
```
### Run in container
To run the Daly BMS application inside a Docker container, execute the following commands. This will start the container in detached mode, allowing the application to run in the background.
```bash
cd container/run
docker compose up -d
```
### Debug in container
For debugging purposes, you can access the container's shell. This allows you to interact with the application directly, inspect logs, and troubleshoot issues. the source is mounted on your container and you can edit with your favourite ide.
```bash
cd container/debug
docker compose up -d
docker compose exec bms bash
```

### Obtaining DEB Files
To generate Debian packages:
```bash
cd container/debs
docker compose up --build
```

### Pass unitary tests
To run the unitary tests:
```bash
cd container/test
docker compose build
```

## Additional information

This package includes an udev rule to find the USB device using its model and vendor identifiers. This is useful to bind the device to a specific port, independently of the actual physical port where the device is plugged. To add this udev rule to your system, just copy it into your /etc/udev/rules.d directory.

