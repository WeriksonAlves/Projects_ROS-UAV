import rospy
from ..ros.DroneSensors import DroneSensors


class DroneSensorManager:
    def __init__(self, sensors):
        self.sensors = sensors

    def update_sensors(self, raw_data):
        sensor_list = self.sensors.extract_sensor_values(raw_data)
        for sensor_name, sensor_value, sensor_enum, _ in sensor_list:
            if sensor_name:
                self.sensors.update(sensor_name, sensor_value, sensor_enum)
            else:
                rospy.logwarn("Received an empty sensor name.")

    def get_sensor_data(self):
        """Returns current sensor data in a structured format."""
        return {
            "altitude": self.sensors.get_altitude(),
            "battery": self.sensors.get_battery(),
            "flying_state": self.sensors.flying_state,
            # Add additional sensor data as required
        }
