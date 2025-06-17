import threading
import rospy

from robotnik_msgs.srv import SetInt16, SetInt16Response, SetInt16Request

try:
    from rcomponent import RComponent
except ImportError:
    from rcomponent.rcomponent import RComponent

from robotnik_msgs.msg import BatteryStatus
from daly_bms_msgs.msg import *

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
        self._complete_status = CompleteStatus()
        self._last_battery_state = 'Unknown'
        self._time_init_charging = rospy.Time.now()
        self._last_discharge_value = 3.0
        self._read_error_count = 0
        self._reconnect_delay = 1.0


    def ros_read_params(self):
        RComponent.ros_read_params(self)

        self._port = rospy.get_param('~serial_port', "/dev/ttyUSB_BMS")
        self._set_soc_service_name = rospy.get_param('~set_soc_service_name', "~set_soc")

    def ros_setup(self):
        self._battery_status_pub = rospy.Publisher("~data", BatteryStatus, queue_size=10)
        self._complete_status_pub = rospy.Publisher("~status", CompleteStatus, queue_size=10)
        self._reading_timer = RepeatTimer(self._publish_state_timer, self.read)

        self._set_soc_service_server = rospy.Service(self._set_soc_service_name, SetInt16, self._set_soc_cb)
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
        self._complete_status_pub.unregister()

        RComponent.ros_shutdown(self)
        
    def handle_read_error(self):
        rospy.logwarn("Skipping current read cycle: Driver failed to return data")
        self._read_error_count += 1
        if self._read_error_count > 10:
            rospy.logwarn("Too many read errors, reconnecting driver")
            self._driver.disconnect()
            # Wait before reconnecting to avoid flooding the serial port
            rospy.logwarn(f"Sleeping {self._reconnect_delay:.1f}s before reconnecting...")
            rospy.sleep(self._reconnect_delay)
            rospy.logwarn("Reconnecting driver...")
            self._driver.connect(self._port)
            # Reset read error counter and reconnect delay
            self._read_error_count = 0
            self._reconnect_delay = min(self._reconnect_delay + 1.0, 30.0) # Increase delay up to a maximum of 30 seconds

    def read(self):
        try:
          soc_data = self._driver.get_soc()
          mosfet_data = self._driver.get_mosfet_status()
          cells_data = self._driver.get_cell_voltages()
          cell_voltage_range = self._driver.get_cell_voltage_range()
          temperature_range = self._driver.get_temperature_range()
          status = self._driver.get_status()
          temperatures = self._driver.get_temperatures()
          balancing_status = self._driver.get_balancing_status()
          errors = self._driver.get_errors()
          
        except:
          self.handle_read_error()
          return

        if soc_data == False or mosfet_data == False or cells_data == False or \
            cell_voltage_range == False or temperature_range == False or \
            status == False or temperatures == False or balancing_status == False or \
            errors == False:
          self.handle_read_error()
          return

        self._read_error_count = 0 #Reset read error counter
        self._reconnect_delay = 1.0 # Reset reconnect delay
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
            remaining_hours = round(mosfet_data['capacity_ah']/self._last_discharge_value, 3)
        else:
            remaining_hours = 0

        self._battery_status.time_remaining = int(max(0, remaining_hours*60)) # remaining_hours is negative in certain cases
        self._last_battery_state = mosfet_data['mode']

        self._battery_status.cell_voltages = list(cells_data.values())
        self._complete_status.cell_voltages = list(cells_data.values())
        # Fill complete status message
        # Soc
        self._complete_status.soc.total_voltage = soc_data['total_voltage']
        self._complete_status.soc.current = soc_data['current']
        self._complete_status.soc.soc_percent = soc_data['soc_percent']
        # CellVoltageRange
        self._complete_status.cell_voltage_range.highest_voltage = cell_voltage_range['highest_voltage']
        self._complete_status.cell_voltage_range.highest_cell = cell_voltage_range['highest_cell']
        self._complete_status.cell_voltage_range.lowest_voltage = cell_voltage_range['lowest_voltage']
        self._complete_status.cell_voltage_range.lowest_cell = cell_voltage_range['lowest_cell']
        # TemperatureRange
        self._complete_status.temperature_range.highest_temperature = temperature_range['highest_temperature']
        self._complete_status.temperature_range.highest_sensor = temperature_range['highest_sensor']
        self._complete_status.temperature_range.lowest_temperature = temperature_range['lowest_temperature']
        self._complete_status.temperature_range.lowest_sensor = temperature_range['lowest_sensor']
        # MosfetStatus
        self._complete_status.mosfet_status.mode = mosfet_data['mode']
        self._complete_status.mosfet_status.charging_mosfet = mosfet_data['charging_mosfet']
        self._complete_status.mosfet_status.discharging_mosfet = mosfet_data['discharging_mosfet']
        self._complete_status.mosfet_status.capacity_ah = mosfet_data['capacity_ah']
        # Status
        self._complete_status.status.cells = status['cells']
        self._complete_status.status.temperature_sensors = status['temperature_sensors']
        self._complete_status.status.charger_running = status['charger_running']
        self._complete_status.status.load_running = status['load_running']
        self._complete_status.status.states = []
        for state in status['states'].keys():
            new_state = State()
            new_state.name = state
            new_state.value = status["states"][state]
            self._complete_status.status.states.append(new_state)
        self._complete_status.status.cycles = status['cycles']
        # Temperatures
        self._complete_status.temperatures = temperatures.values()
        # BalancingStatus
        if len(balancing_status.keys()) == 1:
            self._complete_status.balancing_status.status = list(balancing_status.keys())[0]
            self._complete_status.balancing_status.description = list(balancing_status.values())[0]
        else:
            self._complete_status.balancing_status = BalancingStatus()
        ## Errors
        self._complete_status.errors = errors


    def ros_publish(self):
        self._battery_status_pub.publish(self._battery_status)
        self._complete_status_pub.publish(self._complete_status)

    def _set_soc_cb(self, request: SetInt16Request):
        response = SetInt16Response()

        target_value = request.data.data
        if target_value < 0.0:
            msg = "The specified value (%d) cannot be lower than 0." % target_value
            response.ret.message = msg
            rospy.logerr("%s::_set_soc_cb:: %s" % (self._node_name, msg))
        elif target_value > 100.0:
            msg = "The specified value (%d) cannot be higher than 100." % target_value
            response.ret.message = msg
            rospy.logerr("%s::_set_soc_cb:: %s" % (self._node_name, msg))
        else:
            self._driver.set_soc(target_value)
            msg = "SOC set to %d." % target_value
            response.ret.success = True
            response.ret.message = msg
            rospy.loginfo("%s::_set_soc_cb:: %s" % (self._node_name, msg))

        return response