import rclpy
from rclpy.node import Node
from robotnik_msgs.msg import BatteryStatus
from dalybms import DalyBMS as DalyBMSDriver


class DalyBMS(Node):
    def __init__(self):
        super().__init__("daly_bms")
        self._driver: DalyBMSDriver = DalyBMSDriver()
        self._battery_status = BatteryStatus()
        self._last_battery_state = "Unknown"
        self._time_init_charging = self.get_clock().now()
        self._last_discharge_value = 3.0

    def ros_read_params(self):
        self.declare_parameter("~serial_port", "/dev/ttyUSB_BMS")
        if self.get_parameter("~serial_port").value == None:
            self.get_logger().warn(
                "No serial port provided, using default: /dev/ttyUSB_BMS"
            )
            self._port = "/dev/ttyUSB_BMS"
        else:
            self._port = self.get_parameter("~serial_port").value

    def ros_setup(self):
        self._battery_status_pub = self.create_publisher(BatteryStatus, "data", 10)
        timer_period: float = 1  # seconds
        self._reading_timer = self.create_timer(timer_period, self.read)
        self._publishing_timer = self.create_timer(timer_period, self.publish)

    def setup(self):
        self.ros_read_params()
        self._driver.connect(self._port)
        self.ros_setup()

    def read(self):
        try:
            soc_data = self._driver.get_soc()
            mosfet_data = self._driver.get_mosfet_status()
            cells_data = self._driver.get_cell_voltages()
        except:
            self.get_logger().warn(
                "Skipping current read cycle: Driver failed to return data"
            )
            return

        if soc_data == False or mosfet_data == False or cells_data == False:
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
            self._last_discharge_value = self._battery_status.current

        elif mosfet_data["mode"] == "charging" or mosfet_data["mode"] == "stationary":
            if (
                self._last_battery_state == "Unknown"
                or self._last_battery_state == "discharging"
            ):
                self._time_init_charging = self.get_clock().now().to_msg().sec

            self._battery_status.is_charging = True
            elapsed_time = (
                self.get_clock().now().to_msg().sec - self._time_init_charging
            ) / 60
            elapsed_time = int(elapsed_time)

            self._battery_status.time_charging = elapsed_time

        # _last_discharge_value is negative in certain cases
        if self._last_discharge_value != 0:
            remaining_hours = round(
                mosfet_data["capacity_ah"] / self._last_discharge_value, 0
            )
        else:
            remaining_hours = 0

        self._battery_status.time_remaining = max(
            0, int(remaining_hours) * 60
        )  # remaining_hours is negative in certain cases
        self._last_battery_state = mosfet_data["mode"]

        self._battery_status.cell_voltages = list(cells_data.values())

    def publish(self):
        self._battery_status_pub.publish(self._battery_status)
