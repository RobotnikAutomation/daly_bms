import threading
import rospy

from rcomponent.rcomponent import RComponent
from robotnik_msgs.msg import BatteryStatus

from daly_bms_driver import DalyBMSDriver

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
        self._reading_timer = threading.Timer(self._publish_state_timer, self.read)
        
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
        data = self._driver.get_soc()
        #print(self._driver.get_all())

        self._battery_status.level = data['soc_percent']
        self._battery_status.voltage = data['total_voltage']
        self._battery_status.current = data['current']

        data = self._driver.get_mosfet_status()
        
        if data['mode'] == 'discharging':
            self._battery_status.is_charging = False
            self._battery_status.time_charging = 0
            self._last_discharge_value = self._battery_status.current
        
        elif data['mode'] == 'charging':
        
            if self._last_battery_state == 'Unknown' or self._last_battery_state == 'discharging':
                self._time_init_charging = rospy.Time.now().secs


            self._battery_status.is_charging = True
            elapsed_time = (rospy.Time.now().secs - self._time_init_charging)/60
            elapsed_time = int(elapsed_time)

            self._battery_status.time_charging = elapsed_time

        remaining_hours = round(data['capacity_ah']/self._last_discharge_value, 0)
        self._battery_status.time_remaining = int(remaining_hours)*60
        self._last_battery_state = data['mode']

        self._reading_timer = threading.Timer(self._publish_state_timer, self.read)
        self._reading_timer.start()
        
    
    def ros_publish(self):
        self._battery_status_pub.publish(self._battery_status)



