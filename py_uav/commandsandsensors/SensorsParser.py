"""
Purpose: Retrieves data from the Bebop2 drone's sensors, including GPS,
attitude, speed, and battery levels. Organizes this data into a structured
dictionary.
"""

import time
from typing import Callable, Optional, Dict, Any
from dataclasses import dataclass
from ..ros.Sensors import Sensors


@dataclass
class SensorFlags:
    """Flags to indicate various sensor status changes."""
    relative_move_ended: bool = False
    camera_move_ended_tilt: bool = False
    camera_move_ended_pan: bool = False
    flat_trim_changed: bool = False
    max_altitude_changed: bool = False
    max_distance_changed: bool = False
    no_fly_over_max_distance: bool = False
    max_tilt_changed: bool = False
    max_pitch_roll_rotation_speed_changed: bool = False
    max_vertical_speed_changed: bool = False
    max_rotation_speed_changed: bool = False
    hull_protection_changed: bool = False
    outdoor_mode_changed: bool = False
    picture_format_changed: bool = False
    auto_white_balance_changed: bool = False
    exposition_changed: bool = False
    saturation_changed: bool = False
    timelapse_changed: bool = False
    video_stabilization_changed: bool = False
    video_recording_changed: bool = False
    video_framerate_changed: bool = False
    video_resolutions_changed: bool = False


class SensorsParser:
    """
    Parses and manages sensor data from the Bebop2 drone, with methods to set
    up user-defined callbacks and update sensor states. Converts raw data to
    SI units where applicable.
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(SensorsParser, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes the SensorsParser object with drone type and update
        frequency.

        :param drone_type: Type of the drone.
        :param frequency: Frequency for updating sensor data (in Hz).
        """
        if hasattr(self, 'initialized') and self.initialized:
            return
        self.initialized = True

        self.drone_type = drone_type
        self.update_period = 1 / frequency
        self.last_update_time = time.time()

        self.sensor_data: Dict[str, Any] = {}
        self.flags = SensorFlags()
        self.battery_level = 100
        self.user_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        self.sensors = Sensors(drone_type)

    def _should_update(self) -> bool:
        """Check if the time interval has passed to update sensor data."""
        current_time = time.time()
        if current_time - self.last_update_time >= self.update_period:
            self.last_update_time = current_time
            return True
        return False

    def _convert_to_si_units(self, raw_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert raw sensor data into SI units for standardized usage.

        :param raw_data: Dictionary of raw sensor data.
        :return: Dictionary of data converted to SI units.
        """
        return {
            'speed_mps': raw_data.get('speed', 0) * 0.27778,  # [m/s]
            'altitude_m': raw_data.get('altitude', 0)  # [m]
        }

    def update_sensors(self) -> None:
        """
        Retrieves and converts sensor data to SI units at specified intervals.
        Calls user-defined callback if set.
        """
        if self._should_update():
            raw_data = self.sensors.get_raw_sensor_data()
            self.sensor_data = self._convert_to_si_units(raw_data)
            if self.user_callback:
                self.user_callback(self.sensor_data)

    def set_user_callback(self, callback_function: Callable, *args) -> None:
        """
        Sets the user-defined callback function, which is invoked every time
        sensors are updated.

        :param callback_function: Function to be called on sensor update.
        :param args: Additional arguments for the callback function.
        """
        self.user_callback = lambda data: callback_function(data, *args)

    def update_sensor_flag(self, sensor_name: str, sensor_value: Any,
                           sensor_enum: Dict[str, Dict[Any, str]]) -> None:
        """
        Updates sensor data dictionary and flags based on sensor name, value,
        and enums.

        :param sensor_name: Name of the sensor.
        :param sensor_value: New value of the sensor.
        :param sensor_enum: Enum dictionary for interpreting sensor values.
        """
        if not sensor_name:
            raise ValueError("Sensor name cannot be empty.")

        # Update sensor based on enums or as a regular sensor value
        if (sensor_name, "enum") in sensor_enum:
            self.sensor_data[sensor_name] = sensor_enum[(
                sensor_name, "enum")].get(sensor_value, "UNKNOWN_ENUM_VALUE")
        else:
            self.sensor_data[sensor_name] = sensor_value

        # Map sensor names to flags for specific states
        flag_map = {
            "FlyingStateChanged_state": 'relative_move_ended',
            "PilotingState_FlatTrimChanged": 'flat_trim_changed',
            "moveByEnd_dX": 'relative_move_ended',
            "OrientationV2_tilt": 'camera_move_ended_tilt',
            "OrientationV2_pan": 'camera_move_ended_pan',
            "MaxAltitudeChanged_current": 'max_altitude_changed',
            "MaxDistanceChanged_current": 'max_distance_changed',
            "NoFlyOverMaxDistanceChanged_shouldNotFlyOver": 'no_fly_over_max_distance',
            "MaxTiltChanged_current": 'max_tilt_changed',
            "MaxPitchRollRotationSpeedChanged_current": 'max_pitch_roll_rotation_speed_changed',
            "MaxVerticalSpeedChanged_current": 'max_vertical_speed_changed',
            "MaxRotationSpeedChanged_current": 'max_rotation_speed_changed',
            "HullProtectionChanged_present": 'hull_protection_changed',
            "OutdoorChanged_present": 'outdoor_mode_changed',
            "BatteryStateChanged_battery_percent": 'battery_level',
            "PictureFormatChanged_type": 'picture_format_changed',
            "AutoWhiteBalanceChanged_type": 'auto_white_balance_changed',
            "ExpositionChanged_value": 'exposition_changed',
            "SaturationChanged_value": 'saturation_changed',
            "TimelapseChanged_enabled": 'timelapse_changed',
            "VideoStabilizationModeChanged_mode": 'video_stabilization_changed',
            "VideoRecordingModeChanged_mode": 'video_recording_changed',
            "VideoFramerateChanged_framerate": 'video_framerate_changed',
            "VideoResolutionsChanged_type": 'video_resolutions_changed'
        }

        if sensor_name in flag_map:
            setattr(self.flags, flag_map[sensor_name], True)

        # Call user callback if defined
        if self.user_callback:
            self.user_callback(self.sensor_data)

    def __str__(self) -> str:
        return f"Bebop2 Sensors Data: {self.sensor_data}"
