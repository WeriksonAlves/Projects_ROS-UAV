import rospy
from .DroneInformation import DroneInformation
from typing import Any, Dict


class DroneSensorManager:
    """
    Manages the updating and retrieval of sensor data for the drone.
    """

    def __init__(self, sensors: DroneInformation):
        """
        Initializes the DroneSensorManager with an instance of
        DroneInformation.

        :param sensors: DroneInformation instance to interact with the drone's
                        sensor data.
        """
        self.sensors = sensors

    def update_sensors(self, raw_data: Any) -> None:
        """
        Updates sensor values based on the provided raw data from the drone.

        :param raw_data: Raw data received from the drone's sensors.
        """
        sensor_list = self.sensors.extract_sensor_values(raw_data)
        for sensor_name, sensor_value, sensor_enum, _ in sensor_list:
            if sensor_name:
                self.sensors.update(sensor_name, sensor_value, sensor_enum)
            else:
                rospy.logwarn(
                    "Received an empty sensor name during sensor update.")

    def get_sensor_data(self) -> Dict[str, Any]:
        """
        Retrieves current sensor data in a structured format.

        :return: Dictionary containing the latest sensor data.
        """
        return {
            "altitude": self.sensors.get_altitude(),
            "battery": self.sensors.get_battery(),
            "flying_state": self.sensors.flying_state,
            # Add additional sensor data fields as necessary
        }
