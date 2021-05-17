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
        
    
    def ros_read_params(self):
        RComponent.ros_read_params(self)
        
        self._port = rospy.get_param('~serial_port', "/dev/ttyUSB0")
    
    def ros_setup(self):
        self._battery_status_pub = rospy.Publisher("status", BatteryStatus, queue_size=10)
        self._reading_timer = threading.Timer(self._publish_state_timer, self.read)
        self._reading_timer.start()
        
        RComponent.ros_setup(self)
    
    def setup(self):
        self._driver.connect(self._port)

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
        else:
            self._battery_status.is_charging = True

        self._reading_timer = threading.Timer(self._publish_state_timer, self.read)
        self._reading_timer.start()
        
    
    def ros_publish(self):
        self._battery_status_pub.publish(self._battery_status)



