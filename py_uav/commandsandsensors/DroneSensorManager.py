import rospy
from typing import Any, Optional
from ..ros.DroneSensors import DroneSensors


class DroneSensorManager:
    """
    Manages and processes sensor data from the drone, making it available
    for system-wide access.
    """
    _instance = None
    VALID_SENSOR_KEYS = [
        'position', 'orientation', 'speed_linear', 'speed_angular',
        'gps_position', 'joint_state', 'altitude', 'attitude', 'speed',
        'flying_state', 'battery_level', 'wifi_signal'
    ]

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

        # Initialize default sensor data
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
            'battery_level': 100.0,
            'wifi_signal': 0.0
        }

        self._initialized = True

    def get_sensor_data(self, sensor_key: str) -> Optional[Any]:
        """
        Retrieves the specified sensor data.

        :param sensor_key: Key representing the desired sensor data.
        :return: Sensor data value, or None if key does not exist.
        """
        if sensor_key not in self.VALID_SENSOR_KEYS:
            rospy.logwarn("Invalid sensor key requested: %s", sensor_key)
            return None
        return self.sensors_data.get(sensor_key)

    def get_all_sensor_data(self) -> dict:
        """
        Retrieves all sensor data as a dictionary.

        :return: Dictionary with all sensor data.
        """
        return self.sensors_data

    def update_all_sensors(self) -> None:
        """
        Updates all sensor data values from the drone and stores them in
        `sensors_data`.
        """
        try:
            new_data = self.sensors.get_processed_sensor_data()
            if new_data:
                self.sensors_data.update(new_data)
        except Exception as e:
            rospy.logerr(f"Error updating all sensors: {e}")

    def update_sensor(self, sensor_key: str) -> None:
        """
        Updates the specified sensor data value.

        :param sensor_key: Key representing the sensor data to update.
        """
        if sensor_key not in self.VALID_SENSOR_KEYS:
            rospy.logwarn("Invalid sensor key: %s. Update aborted.",
                          sensor_key)
            return

        try:
            updated_data = self.sensors.get_processed_sensor_data()
            if updated_data and sensor_key in updated_data:
                self.sensors_data[sensor_key] = updated_data[sensor_key]
            else:
                rospy.logwarn("Sensor data for key '%s' could not be updated.",
                              sensor_key)
        except Exception as e:
            rospy.logerr(f"Error updating sensor {sensor_key}: {e}")

    def validate_and_smooth_data(self, sensor_key: str, new_value: Any) -> Any:
        """
        Validates and applies smoothing to the sensor data if needed.

        :param sensor_key: Key representing the sensor data.
        :param new_value: The new value to process.
        :return: Smoothed or validated data value.
        """
        # Placeholder for data smoothing or validation logic
        if sensor_key in self.sensors_data and isinstance(new_value, (int,
                                                                      float,
                                                                      list)):
            current_value = self.sensors_data[sensor_key]
            # For example, averaging the new value with the current one
            if isinstance(new_value, list) and isinstance(current_value, list):
                smoothed_value = [(nv + cv) / 2 for nv, cv in zip(new_value,
                                                                  current_value
                                                                  )]
            elif isinstance(new_value, (int, float)) and isinstance(
                    current_value, (int, float)):
                smoothed_value = (new_value + current_value) / 2
            else:
                smoothed_value = new_value
            return smoothed_value
        return new_value
