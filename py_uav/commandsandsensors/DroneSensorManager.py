import rospy
from typing import Any, Optional
from ..ros.DroneSensors import DroneSensors


class DroneSensorManager:
    """
    Manages and processes sensor data from the drone, making it available
    for system-wide access.
    """
    _instance = None

    def __new__(cls, *args, **kwargs):
        """
        Ensures only one instance of DroneSensorManager is created (Singleton).
        """
        if cls._instance is None:
            cls._instance = super(DroneSensorManager, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: float = 30.0) -> None:
        """
        Initializes DroneSensorManager with an instance of DroneSensors.

        :param drone_type: Type of the drone being used.
        :param frequency: Frequency of sensor updates (Hz).
        """
        if hasattr(self, "_initialized") and self._initialized:
            return  # Avoid reinitializing in Singleton pattern

        self.drone_type = drone_type
        self.frequency = frequency
        self.sensors = DroneSensors(self.drone_type, self.frequency)

        self.sensors_data = {
            'position': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0],
            'speed_linear': [0.0, 0.0, 0.0],
            'speed_angular': [0.0, 0.0, 0.0],
            'gps_position': [0.0, 0.0, 0.0],
            'joint_state': [0.0, 0.0, 0.0],
            'altitude': 0.0,
            'attitude': [0.0, 0.0, 0.0],
            'speed': [0.0, 0.0, 0.0],
            'flying_state': 0,
            'battery_level': 0.0,
            'wifi_signal': 0.0
        }

        self._initialized = True

    def get_sensor_data(self, sensor_key: str) -> Optional[Any]:
        """
        Retrieves the specified sensor data.

        :param sensor_key: Key representing the desired sensor data.
        :return: Sensor data value, or None if key does not exist.
        """
        return self.sensors_data.get(sensor_key)

    def update_all_sensors(self) -> None:
        """
        Updates all sensor data values from the drone and stores them in
        `sensors_data`.
        """
        new_data = self.sensors.get_processed_sensor_data()
        if new_data:
            self.sensors_data.update(new_data)

    def update_sensor(self, sensor_key: str) -> None:
        """
        Updates the specified sensor data value.

        :param sensor_key: Key representing the sensor data to update.
        """
        if sensor_key not in self.sensors_data:
            rospy.logwarn("Invalid sensor key: %s. Update aborted.",
                          sensor_key)
            return

        updated_data = self.sensors.get_processed_sensor_data()
        if updated_data and sensor_key in updated_data:
            self.sensors_data[sensor_key] = updated_data[sensor_key]
        else:
            rospy.logwarn("Sensor data for key '%s' could not be updated.",
                          sensor_key)
