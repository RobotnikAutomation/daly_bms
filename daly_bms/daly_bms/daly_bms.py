# Copyright 2024 Robotnik Automation S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# @maintanier Guillem Gari  <ggari@robotnik.es> Robotnik Automation S.L.
"""
Daly BMS ROS2 Node for battery status monitoring and publishing.

This module implements a ROS2 node that interfaces with a Daly Battery
Management System (BMS) through a serial connection. It reads battery data
and publishes it as a BatteryStatus message.

The node periodically reads data from the BMS, including state of charge,
voltage, current, and individual cell voltages. It also tracks charging
status and estimates remaining time.

Classes:
    DalyBMSConfig: Configuration dataclass for the DalyBMS node.
    DalyBMS: Main ROS2 node class for interfacing with the Daly BMS.

Constants:
    BMS_PORT: Default serial port for BMS communication.

Dependencies:
    - rclpy
    - robotnik_msgs
    - dalybms
    - serial
"""

from dataclasses import dataclass, field
from typing import Any

import serial

from rclpy.node import Node
from robotnik_msgs.msg import BatteryStatus
from dalybms import DalyBMS as DalyBMSDriver


BMS_PORT = "/dev/ttyUSB_BMS"


@dataclass
class DalyBMSConfig:
    """
    Configuration dataclass for the DalyBMS node.

    This class holds configuration parameters and runtime data for the
    DalyBMS node.

    Attributes
    ----------
    port : str
        Serial port for BMS communication.
    last_battery_state : str
        Last known battery state (e.g., "charging", "discharging").
    time_init_charging : Any
        Timestamp when charging started.
    reading_timer : Any
        Timer object for periodic BMS data reading.
    publishing_timer : Any
        Timer object for periodic data publishing.
    last_discharge_value : float
        Last recorded discharge current value.
    timer_period : float
        Period for reading and publishing timers in seconds.

    """

    port: str = ""
    last_battery_state: str = "Unknown"
    time_init_charging: Any = field(default=None)
    reading_timer: Any = field(default=None)
    publishing_timer: Any = field(default=None)
    last_discharge_value: float = 3.0
    timer_period: float = 1.0


class DalyBMS(Node):
    """
    ROS2 Node for interfacing with a Daly Battery Management System.

    This class implements a ROS2 node that communicates with a Daly BMS,
    reads battery data, and publishes it as a BatteryStatus message.

    Attributes
    ----------
    _driver : DalyBMSDriver
        Instance of the Daly BMS driver for low-level communication.
    _battery_status : BatteryStatus
        ROS message object to store and publish battery status data.
    _config : DalyBMSConfig
        Configuration object holding node parameters and runtime data.
    _battery_status_pub : Publisher
        ROS publisher for BatteryStatus messages.

    Methods
    -------
    ros_read_params()
        Read ROS parameters for node configuration.
    ros_setup()
        Set up ROS publishers and timers.
    setup()
        Initialize the node, including parameter reading and ROS setup.
    read()
        Read data from the BMS and update the battery status.
    publish()
        Publish the current battery status.

    """

    def __init__(self):
        """
        Initialize the DalyBMS node.

        This constructor sets up the basic node structure, initializes
        class attributes, and prepares the node for operation.

        The node is not fully set up in the constructor. The `setup()`
        method should be called after initialization to complete the setup.
        """
        super().__init__("daly_bms")
        self._driver: DalyBMSDriver = DalyBMSDriver()
        self._battery_status = BatteryStatus()
        self._config = DalyBMSConfig()
        self._config.time_init_charging = self.get_clock().now()
        self._battery_status_pub = None

    def ros_read_params(self):
        """
        Read ROS parameters for node configuration.

        This method declares and reads the serial port parameter. If no
        parameter is provided, it uses the default BMS_PORT.

        """
        self.declare_parameter("~serial_port", BMS_PORT)
        if self.get_parameter("~serial_port").value is None:
            self.get_logger().warn(
                f"No serial port provided, using default: {BMS_PORT}"
            )
            self._config.port = BMS_PORT
        else:
            self._config.port = self.get_parameter("~serial_port").value

    def ros_setup(self):
        """
        Set up ROS publishers and timers.

        This method creates the battery status publisher and sets up
        timers for periodic reading and publishing of BMS data.
        """
        self._battery_status_pub = self.create_publisher(
            BatteryStatus, "~/data",
            10
        )
        self._config.reading_timer = self.create_timer(
            self._config.timer_period,
            self.read
        )
        self._config.publishing_timer = self.create_timer(
            self._config.timer_period,
            self.publish
        )

    def setup(self):
        """
        Initialize the node, including parameter reading and ROS setup.

        This method should be called after node creation to complete
        the setup process. It reads parameters, connects to the BMS,
        and sets up ROS components.
        """
        self.ros_read_params()
        self._driver.connect(self._config.port)
        self.ros_setup()

    def read(self):
        """
        Read data from the BMS and update the battery status.

        This method attempts to read SOC, MOSFET status, and cell voltages
        from the BMS. It updates the battery status message with the new data.

        Raises
        ------
        SerialException
            If there's an error communicating with the BMS.

        """
        try:
            soc_data = self._driver.get_soc()
            mosfet_data = self._driver.get_mosfet_status()
            cells_data = self._driver.get_cell_voltages()
        except serial.SerialException as excp:
            self.get_logger().warn(
                "Skipping current read cycle: Driver failed to return data",
                f"{excp}"
            )
            return

        if soc_data is False or mosfet_data is False or cells_data is False:
            self.get_logger().warn(
                "Skipping current read cycle: Driver failed to return data"
            )
            return

        self._battery_status.level = soc_data["soc_percent"]
        self._battery_status.voltage = soc_data["total_voltage"]
        self._battery_status.current = soc_data["current"]

        if mosfet_data["mode"] == "discharging":
            self._battery_status.is_charging = False
            self._battery_status.time_charging = 0
            self._config.last_discharge_value = self._battery_status.current

        elif (
            mosfet_data["mode"] == "charging"
            or mosfet_data["mode"] == "stationary"
        ):
            if self._config.last_battery_state in ('Unknown', 'discharging'):
                self._config.time_init_charging = (
                    self.get_clock()
                    .now().to_msg().sec
                )

            self._battery_status.is_charging = True
            elapsed_time = (
                self.get_clock().now().to_msg().sec
                - self._config.time_init_charging
            ) / 60
            elapsed_time = int(elapsed_time)

            self._battery_status.time_charging = elapsed_time

        # _last_discharge_value is negative in certain cases
        if self._config.last_discharge_value != 0:
            remaining_hours = round(
                mosfet_data["capacity_ah"] / self._config.last_discharge_value,
                0
            )
        else:
            remaining_hours = 0

        self._battery_status.time_remaining = max(
            0, int(remaining_hours) * 60
        )  # remaining_hours is negative in certain cases
        self._config.last_battery_state = mosfet_data["mode"]

        self._battery_status.cell_voltages = list(cells_data.values())

    def publish(self):
        """
        Publish the current battery status.

        This method publishes the latest battery status data to the
        designated ROS topic.

        """
        self._battery_status_pub.publish(self._battery_status)
