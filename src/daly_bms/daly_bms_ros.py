import threading
import rospy

from rcomponent.rcomponent import RComponent
from robotnik_msgs.msg import BatteryStatus

from dalybms import DalyBMS as DalyBMSDriver

class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

class DalyBMS(RComponent):
    def __init__(self):
        RComponent.__init__(self)

        self._driver = DalyBMSDriver()
        self._battery_status = BatteryStatus()
        self._last_battery_state = 'Unknown'
        self._time_init_charging = rospy.Time.now()
        self._last_discharge_value = 3.0


    def ros_read_params(self):
        RComponent.ros_read_params(self)

        self._port = rospy.get_param('~serial_port', "/dev/ttyUSB_BMS")

    def ros_setup(self):
        self._battery_status_pub = rospy.Publisher("~data", BatteryStatus, queue_size=10)
        self._reading_timer = RepeatTimer(self._publish_state_timer, self.read)

        RComponent.ros_setup(self)

    def setup(self):
        self._driver.connect(self._port)
        self._reading_timer.start()

        RComponent.setup(self)

    def shutdown(self):
        self._reading_timer.cancel()

        RComponent.shutdown(self)

    def ros_shutdown(self):
        self._battery_status_pub.unregister()

        RComponent.ros_shutdown(self)

    def read(self):
        try:
          soc_data = self._driver.get_soc()
          mosfet_data = self._driver.get_mosfet_status()
          cells_data = self._driver.get_cell_voltages()
        except:
          rospy.logwarn("Skipping current read cycle: Driver failed to return data")
          return

        if soc_data == False or mosfet_data == False or cells_data == False:
          rospy.logwarn("Skipping current read cycle: Driver failed to return data")
          return

        self._battery_status.level = soc_data['soc_percent']
        self._battery_status.voltage = soc_data['total_voltage']
        self._battery_status.current = soc_data['current']

        if mosfet_data['mode'] == 'discharging':
            self._battery_status.is_charging = False
            self._battery_status.time_charging = 0
            self._last_discharge_value = self._battery_status.current

        elif mosfet_data['mode'] == 'charging' or mosfet_data['mode'] == 'stationary':

            if self._last_battery_state == 'Unknown' or self._last_battery_state == 'discharging':
                self._time_init_charging = rospy.Time.now().secs


            self._battery_status.is_charging = True
            elapsed_time = (rospy.Time.now().secs - self._time_init_charging)/60
            elapsed_time = int(elapsed_time)

            self._battery_status.time_charging = elapsed_time

        # _last_discharge_value is negative in certain cases
        if self._last_discharge_value != 0:
            remaining_hours = round(mosfet_data['capacity_ah']/self._last_discharge_value, 0)
        else:
            remaining_hours = 0

        self._battery_status.time_remaining = max(0, int(remaining_hours)*60) # remaining_hours is negative in certain cases
        self._last_battery_state = mosfet_data['mode']

        self._battery_status.cell_voltages = list(cells_data.values())


    def ros_publish(self):
        self._battery_status_pub.publish(self._battery_status)
