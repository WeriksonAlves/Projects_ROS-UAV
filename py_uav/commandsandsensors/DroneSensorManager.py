import rospy
from typing import Any, Optional, Dict, Tuple, Callable
from ..ros.DroneSensors import DroneSensors


class DroneSensorManager:
    """
    Singleton class to manage and process sensor data from the drone, providing
    a centralized interface for sensor access and updates.
    """
    _instance = None

    VALID_SENSOR_KEYS = [
        'position', 'orientation', 'speed_linear', 'speed_angular',
        'gps_position', 'joint_state', 'altitude', 'attitude', 'speed',
        'flying_state', 'battery_level', 'wifi_signal'
    ]

    def __new__(cls, *args, **kwargs):
        """
        Singleton pattern implementation to ensure a single instance.
        """
        if cls._instance is None:
            cls._instance = super(DroneSensorManager, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: float = 30.0) -> None:
        """
        Initializes the DroneSensorManager with a drone type and sensor update
        frequency. Avoids re-initialization in Singleton pattern.

        :param drone_type: The type of the drone being used.
        :param frequency: Frequency of sensor updates (Hz).
        """
        if getattr(self, "_initialized", False):
            return

        self.drone_type = drone_type
        self.frequency = frequency
        self.sensors = DroneSensors(self.drone_type, self.frequency)
        self.user_callback: Optional[Callable] = None
        self.user_callback_args: Optional[Tuple] = None
        self.sensors_data = self._initialize_sensor_data()
        self._initialize_status_flags()
        self._initialized = True

    def _initialize_sensor_data(self) -> Dict[str, Any]:
        """
        Sets up a dictionary to hold default values for each sensor.

        :return: Dictionary of initialized sensor values.
        """
        return {
            'position': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0],
            'speed_linear': [0.0, 0.0, 0.0],
            'speed_angular': [0.0, 0.0, 0.0],
            'gps_position': [0.0, 0.0, 0.0],
            'joint_state': [0.0, 0.0, 0.0],
            'altitude': 0.0,
            'attitude': [0.0, 0.0, 0.0],
            'speed': [0.0, 0.0, 0.0],
            'flying_state': "unknown",
            'battery_level': 100.0,
            'wifi_signal': 0.0
        }

    def _initialize_status_flags(self) -> None:
        """
        Initializes various status flags for drone configuration and state
        changes.
        """
        self.status_flags = [
            "RelativeMoveEnded", "flat_trim_changed", "max_altitude_changed",
            "max_distance_changed", "no_fly_over_max_distance",
            "max_tilt_changed", "max_pitch_roll_rotation_speed_changed",
            "max_vertical_speed_changed", "max_rotation_speed_changed",
            "hull_protection_changed", "outdoor_mode_changed",
            "picture_format_changed", "auto_white_balance_changed",
            "exposition_changed", "saturation_changed", "timelapse_changed",
            "video_stabilization_changed", "video_recording_changed",
            "video_framerate_changed", "video_resolutions_changed"
        ]
        self.CameraMoveEnded = {"tilt": False, "pan": False}
        for flag in self.status_flags:
            setattr(self, flag, False)

    def set_user_callback(self, function: Callable, args: Tuple) -> None:
        """
        Registers a user-defined callback function to be called after each
        sensor data update.

        :param function: The callback function to trigger after updates.
        :param args: Arguments to pass to the callback function.
        """
        self.user_callback = function
        self.user_callback_args = args

    def get_sensor_data(self, sensor_key: str) -> Optional[Any]:
        """
        Retrieves the specified sensor data if valid.

        :param sensor_key: Key representing the desired sensor data.
        :return: Sensor data value, or None if the key is invalid.
        """
        if sensor_key not in self.VALID_SENSOR_KEYS:
            rospy.logwarn(f"Invalid sensor key requested: {sensor_key}")
            return None
        return self.sensors_data.get(sensor_key)

    def get_all_sensor_data(self) -> Dict[str, Any]:
        """
        Retrieves all sensor data as a dictionary.

        :return: Dictionary containing all sensor data.
        """
        return self.sensors_data

    def update_all_sensors(self) -> None:
        """
        Updates all sensor data from the drone and stores them in the internal
        sensor data dictionary.
        """
        try:
            new_data = self.sensors.get_processed_sensor_data()
            if new_data:
                self.sensors_data.update(new_data)
                self._trigger_user_callback()
        except Exception as e:
            rospy.logerr(f"Error updating all sensors: {e}")

    def update_sensor(self, sensor_key: str) -> None:
        """
        Updates a single sensor's data if the key is valid.

        :param sensor_key: Key of the sensor to update.
        """
        if sensor_key not in self.VALID_SENSOR_KEYS:
            rospy.logwarn(f"Invalid sensor key: {sensor_key}. Update aborted.")
            return

        try:
            updated_data = self.sensors.get_processed_sensor_data()
            if updated_data and sensor_key in updated_data:
                self.sensors_data[sensor_key] = updated_data[sensor_key]
                self._trigger_user_callback()
            else:
                rospy.logwarn(f"Sensor data for key '{sensor_key}' could not "
                              f"be updated.")
        except Exception as e:
            rospy.logerr(f"Error updating sensor {sensor_key}: {e}")

    def validate_and_smooth_data(self, sensor_key: str, new_value: Any) -> Any:
        """
        Validates and optionally applies smoothing to new sensor data values.

        :param sensor_key: Key of the sensor data.
        :param new_value: New value to process.
        :return: Smoothed or validated data value.
        """
        current_value = self.sensors_data.get(sensor_key)
        if not current_value:
            return new_value

        # Simple averaging for demonstration, can be enhanced
        if isinstance(new_value, (int, float)) and isinstance(current_value, (
                int, float)):
            return (new_value + current_value) / 2
        elif isinstance(new_value, list) and isinstance(current_value, list):
            return [(nv + cv) / 2 for nv, cv in zip(new_value, current_value)]
        return new_value

    def _update_internal_state(self, sensor_name: str, sensor_value: Any
                               ) -> None:
        """
        Updates internal flags based on specific sensor updates.

        :param sensor_name: The name of the sensor being updated.
        :param sensor_value: The updated value.
        """
        self.state_flags = {
            "FlyingStateChanged_state": ("flying_state", sensor_value),
            "PilotingState_FlatTrimChanged": ("flat_trim_changed", True),
            "moveByEnd_dX": ("RelativeMoveEnded", True),
            "OrientationV2_tilt": ("CameraMoveEnded", {"tilt": True}),
            "OrientationV2_pan": ("CameraMoveEnded", {"pan": True}),
            "MaxAltitudeChanged_current": ("max_altitude_changed", True),
            "MaxDistanceChanged_current": ("max_distance_changed", True),
            "BatteryStateChanged_battery_percent": ("battery_level",
                                                    sensor_value),
            # Add more mappings as needed
        }

        if sensor_name in self.state_flags:
            attr_name, attr_value = self.state_flags[sensor_name]
            if isinstance(attr_value, dict):
                # Handle dict updates for tilt/pan
                self.CameraMoveEnded.update(attr_value)
            else:
                setattr(self, attr_name, attr_value)

    def _trigger_user_callback(self) -> None:
        """
        Executes the user-defined callback function, if provided.
        """
        if self.user_callback:
            self.user_callback(*self.user_callback_args)

    def __str__(self) -> str:
        """
        String representation of the current sensor data.

        :return: Formatted string of sensor data.
        """
        return f"Drone sensor data: {self.sensors_data}"
