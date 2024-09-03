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
import time

import serial

from rclpy.node import Node
from robotnik_msgs.msg import BatteryStatus
from dalybms import DalyBMS as DalyBMSDriver


BMS_PORT = "/dev/ttyUSB_BMS"


@dataclass
class DalyBMSReadConfig:
    """
    Configuration dataclass for the DalyBMS read operations and status.

    This class encapsulates various configuration parameters and status
    flags related to reading data from the Daly Battery Management System
    (BMS). It includes settings for timeouts, connection retries, and
    flags indicating the current state of the connection and communication.

    Attributes
    ----------
    read_timeout : float
        Maximum time allowed (in seconds) without receiving data before
        considering the communication as failed. Default is 1.0 second.
    connection_retries : int
        Maximum number of attempts to reconnect to the BMS in case of
        connection failure. Default is 5 attempts.
    reconnect_wait : float
        Initial time to wait (in seconds) between connection retry
        attempts. This may be increased with exponential backoff in the
        connection logic. Default is 1.0 second.
    connected : bool
        Flag indicating whether the serial port is currently connected
        to the BMS. Initially set to False.
    communicating : bool
        Flag indicating whether the node is successfully receiving data
        from the BMS within the read_timeout period. Initially set to
        True.
    last_successful_read : Any
        Timestamp of the last successful data read from the BMS. Used to
        calculate the time elapsed since the last successful communication.
        Initially set to None and updated during read operations.

    Notes
    -----
    - The `connected` flag should be set to True when a successful
      connection is established and to False if the connection is lost.
    - The `communicating` flag should be updated based on whether data
      is being received within the `read_timeout` period.
    - `last_successful_read` should be updated with the current timestamp
      whenever data is successfully read from the BMS.

    """

    read_timeout: float = 10.0
    connection_retries: int = 5
    reconnect_wait: float = 1.0
    connected: bool = False
    communicating: bool = True
    last_successful_read: Any = field(default=None)


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
        self._read_config = DalyBMSReadConfig(
            last_successful_read=self.get_clock().now()
        )
        self._config.time_init_charging = self.get_clock().now()
        self._battery_status_pub = None

    def ros_read_params(self):
        """
        Read ROS parameters for node configuration.

        This method declares and reads the serial port parameter. If no
        parameter is provided, it uses the default BMS_PORT.

        """
        param = "serial_port"
        default_value = BMS_PORT
        self.declare_parameter(
            name=param,
            value=default_value,
        )
        if not self.get_parameter(param).value:
            self.get_logger().warn(
                f"No serial port provided, using default: {default_value}"
            )
            self._config.port = default_value
        self._config.port = self.get_parameter(param).value
        assert isinstance(
            self._config.port,
            str
        ), 'port parameter must be a str'
        self.get_logger().info(f"serial port: {self._config.port}")

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

    def connect_device(self):
        """
        Establish a connection to the Battery Management System (BMS) device.

        This method attempts to connect to the BMS using the configured port.
        It will retry the connection a specified number of times if initial
        attempts fail. The method updates the connection status in the
        _read_config attribute.

        The connection process involves:
        1. Setting the initial connection status to False.
        2. Attempting to connect using the driver's connect method.
        3. Retrying the connection if it fails, with a specified wait time
           between attempts.
        4. Updating the connection status upon successful connection.

        Raises
        ------
        ConnectionError
            If the maximum number of connection retries is reached without
            successfully connecting to the BMS.

        Notes
        -----
        - The number of retries and wait time between retries are specified
          in the _read_config attribute.
        - The method uses exponential backoff for retries, doubling the wait
          time after each failed attempt.
        - Upon successful connection, a log message is generated.

        """
        self._read_config.connected = False
        retries = self._read_config.connection_retries
        wait = self._read_config.reconnect_wait
        while not self._read_config.connected and retries > 0:
            try:
                self._driver.connect(self._config.port)
                self._read_config.connected = True
                self.get_logger().info("Successfully connected to BMS.")
            except serial.SerialException as excp:
                retries -= 1
                self.get_logger().warn(
                    f"Connection attempt {retries} failed: {str(excp)}"
                )
                if retries > 0:
                    self.get_logger().info(f"Retrying in {wait} seconds...")
                    time.sleep(wait)
                    wait *= 1.5  # Exponential backoff
                else:
                    self.get_logger().error(
                        "Max retries reached. Unable to connect to BMS."
                    )
                    raise ConnectionError(
                        "Failed to connect to BMS after maximum retries."
                    ) from excp

    def reconnect_device(self):
        """
        Attempt to reconnect to the Battery Management System (BMS) device.

        This method is called when a communication issue is detected with the
        BMS. It performs the following steps to re-establish the connection:

        1. Log a warning message indicating the reconnection attempt.
        2. Disconnect the current driver connection.
        3. Set the connected status to False.
        4. Attempt to reconnect using the connect_device method.

        The method uses the port specified in the configuration to reconnect.
        If the reconnection is successful, the connected status will be
        updated in the connect_device method.

        Notes
        -----
        - This method does not update the last_successful_read timestamp.
          This is to ensure that if the reconnection doesn't immediately
          result in successful data reads, the check_dataflow method can
          still accurately track the time since the last successful read.
        - The actual connection logic is handled by the connect_device
          method, which may raise exceptions if the connection fails.

        """
        self.get_logger().warn(
            f"Reconnecting serial to port {self._config.port}"
        )
        self._driver.disconnect()
        self._read_config.connected = False
        self.connect_device()
        # self._read_config.last_successful_read = self.get_clock().now()

    def setup(self):
        """
        Initialize the node, including parameter reading and ROS setup.

        This method performs the complete setup process for the DalyBMS node.
        It should be called after node creation to initialize all necessary
        components. The setup process includes:

        1. Reading ROS parameters to configure the node.
        2. Establishing a connection to the BMS device.
        3. Setting up ROS components (publishers, subscribers, etc.).

        The method ensures that all required configurations are in place and
        that the node is ready for operation. It handles the initialization
        sequence, making sure each step is completed before proceeding to the
        next.

        """
        self.ros_read_params()
        self.connect_device()
        self.ros_setup()

    def read(self):
        """
        Read data from the Battery Management System (BMS).

        This method is responsible for retrieving the State of Charge (SOC),
        MOSFET status, and cell voltages from the BMS. It processes this data
        to update the battery status message, ensuring that the latest
        information is available.

        The method performs the following steps:
        1. Check if the serial port is connected. If not, log a warning and
           skip the read cycle.
        2. Verify data flow using `check_dataflow()`. If the data flow is
           interrupted, attempt to reconnect.
        3. Attempt to read SOC, MOSFET status, and cell voltages from the BMS.
        4. If any read operation fails, log a warning, set the communication
           status to False, and skip the current read cycle.
        5. If all read operations are successful, update the communication
           status, record the time of the last successful read, and process
           the data to update the battery status.

        The data processing includes:
        - Updating battery level, voltage, and current.
        - Determining charging status and calculating charging time.
        - Estimating remaining time based on capacity and discharge rate.
        - Updating cell voltage information.

        This method is typically called periodically to keep the battery
        status information up to date.

        Raises
        ------
        SerialException
            If there's an error in communicating with the BMS through the
            serial connection. This exception is caught and logged, and the
            method skips the current read cycle.

        Notes
        -----
        - The method uses a private driver instance (`self._driver`) to
          communicate with the BMS.
        - If any of the read operations return False, it's treated as a
          failure, and the current read cycle is skipped.
        - The processed data is stored in `self._battery_status`, which is
          presumably a ROS message object for publishing battery status.

        """
        if not self._read_config.connected:
            self.get_logger().warn(
                "Serial port is not connected skipping"
            )
            return
        if not self.check_dataflow():
            self.reconnect_device()
        try:
            soc_data = self._driver.get_soc()
            mosfet_data = self._driver.get_mosfet_status()
            cells_data = self._driver.get_cell_voltages()
        except serial.SerialException as excp:
            self._read_config.communicating = False
            self.get_logger().debug(f"{excp}")
            self.get_logger().warn(
                "Skipping current read cycle: Driver failed to return data"
            )
            return
        if soc_data is False or mosfet_data is False or cells_data is False:
            self._read_config.communicating = False
            self.get_logger().warn(
                "Skipping current read cycle: Driver failed to return data"
            )
            return
        self._read_config.communicating = True
        self._read_config.last_successful_read = self.get_clock().now()
        self.process_data(soc_data, mosfet_data, cells_data)

    def check_dataflow(self):
        """
        Check the data flow from the Battery Management System (BMS).

        This method calculates the time elapsed since the last successful data
        read from the BMS. If the elapsed time exceeds the configured read
        timeout, it logs a warning and updates the communication status to
        indicate that data is not being received.

        Returns
        -------
        bool
            True if data is being received within the timeout period, False
            otherwise.

        Notes
        -----
        - The method uses the ROS clock to determine the current time and
          calculate the time since the last successful read.
        - The `communicating` attribute in the `_read_config` is set to False
          if the timeout is exceeded, indicating a failure in data flow.

        """
        current_time = self.get_clock().now()
        time_since_last_read = (
            current_time - self._read_config.last_successful_read
        )
        time_since_last_read_sec = time_since_last_read.nanoseconds / 1e9

        if time_since_last_read_sec > self._read_config.read_timeout:
            self.get_logger().warn(
                f"No data received for {round(time_since_last_read_sec, 2)} "
                f"seconds, exceeding timeout "
                f"{self._read_config.read_timeout}"
            )
            self._read_config.communicating = False
            return False

        return True

    def process_data(
        self,
        soc_data,
        mosfet_data,
        cells_data,
    ):
        """
        Process the data read from the BMS and update the battery status.

        This method takes the raw data from the BMS and updates the battery
        status message with processed information. It handles calculations
        for charging status, time charging, and estimated time remaining.

        Parameters
        ----------
        soc_data : dict
            Dictionary containing State of Charge data including 'soc_percent',
            'total_voltage', and 'current'.
        mosfet_data : dict
            Dictionary containing MOSFET status data including 'mode' and
            'capacity_ah'.
        cells_data : dict
            Dictionary containing voltage data for individual cells.

        Notes
        -----
        This method updates several attributes of `self._battery_status`:
        - level: Battery charge level as a percentage.
        - voltage: Total battery voltage.
        - current: Current flowing through the battery.
        - is_charging: Boolean indicating if the battery is charging.
        - time_charging: Time spent charging in minutes.
        - time_remaining: Estimated time remaining in minutes.
        - cell_voltages: List of individual cell voltages.

        It also updates `self._config.last_battery_state` and
        `self._config.last_discharge_value` for tracking purposes.

        """
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
