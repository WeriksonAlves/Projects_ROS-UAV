import rospy
from .DroneSensorManager import DroneSensorManager
from typing import Callable, Any, Dict, Optional, Tuple


class DroneInformation:
    """
    Manages general and main sensor information for the drone, including
    user-defined callbacks.
    """

    def __init__(self):
        """
        Initializes the DroneInformation class, setting up sensor data and
        callback configurations.
        """
        self.sensor_parser = DroneSensorManager()
        self.user_callback_function: Optional[Callable] = None
        self.user_callback_function_args: Optional[Tuple] = None

        self.sensors_dict: Dict[str, Any] = {}
        self.initialize_status_flags()

        self.flying_state = "unknown"
        self.battery = 100  # Default to full battery

    def initialize_status_flags(self):
        """
        Initializes status flags for various drone properties that indicate
        changes in configuration or state.
        """
        self.RelativeMoveEnded = False
        self.CameraMoveEnded = {"tilt": False, "pan": False}
        self.flat_trim_changed = False
        self.max_altitude_changed = False
        self.max_distance_changed = False
        self.no_fly_over_max_distance = False
        self.max_tilt_changed = False
        self.max_pitch_roll_rotation_speed_changed = False
        self.max_vertical_speed_changed = False
        self.max_rotation_speed_changed = False
        self.hull_protection_changed = False
        self.outdoor_mode_changed = False
        self.picture_format_changed = False
        self.auto_white_balance_changed = False
        self.exposition_changed = False
        self.saturation_changed = False
        self.timelapse_changed = False
        self.video_stabilization_changed = False
        self.video_recording_changed = False
        self.video_framerate_changed = False
        self.video_resolutions_changed = False

    def set_user_callback_function(self, function: Callable, args: Tuple
                                   ) -> None:
        """
        Sets a user-defined callback function, which will be executed whenever
        the sensor data is updated.

        :param function: The callback function to be triggered after updates.
        :param args: Arguments to pass to the callback function.
        """
        self.user_callback_function = function
        self.user_callback_function_args = args

    def update(self, sensor_name: str, sensor_value: Any,
               sensor_enum: Dict[Tuple[str, str], Any]) -> None:
        """
        Updates sensor information based on sensor name, value, and optional
        enum mapping.

        :param sensor_name: The name of the sensor to update.
        :param sensor_value: The new value of the sensor.
        :param sensor_enum: Dictionary mapping sensor enums to readable values.
        """
        if not sensor_name:
            rospy.logwarn("Error: Empty sensor name provided.")
            return

        # Check if sensor uses an enum for its values
        if (sensor_name, "enum") in sensor_enum:
            value = self.get_enum_value(sensor_name, sensor_value, sensor_enum)
        else:
            value = sensor_value

        self.sensors_dict[sensor_name] = value
        self.update_internal_state(sensor_name, value)
        self.trigger_user_callback()

    def get_enum_value(self, sensor_name: str, sensor_value: Any,
                       sensor_enum: Dict[Tuple[str, str], Any]) -> str:
        """
        Retrieves the enum string for a given sensor value, or returns
        'UNKNOWN_ENUM_VALUE' if out of range.

        :param sensor_name: The name of the sensor with an enum.
        :param sensor_value: The index of the enum value.
        :param sensor_enum: Dictionary mapping sensor enums to readable values.
        :return: The string representation of the enum or 'UNKNOWN_ENUM_VALUE'
                 if not found.
        """
        if sensor_value is None or sensor_value >= len(sensor_enum.get(
                (sensor_name, "enum"), [])):
            return "UNKNOWN_ENUM_VALUE"
        return sensor_enum[(sensor_name, "enum")][sensor_value]

    def update_internal_state(self, sensor_name: str, sensor_value: Any
                              ) -> None:
        """
        Updates internal state flags based on specific sensor name-value pairs.

        :param sensor_name: The name of the sensor being updated.
        :param sensor_value: The updated value of the sensor.
        """
        state_flags = {
            "FlyingStateChanged_state": ("flying_state", sensor_value),
            "PilotingState_FlatTrimChanged": ("flat_trim_changed", True),
            "moveByEnd_dX": ("RelativeMoveEnded", True),
            "OrientationV2_tilt": ("CameraMoveEnded", {"tilt": True}),
            "OrientationV2_pan": ("CameraMoveEnded", {"pan": True}),
            "MaxAltitudeChanged_current": ("max_altitude_changed", True),
            "MaxDistanceChanged_current": ("max_distance_changed", True),
            "NoFlyOverMaxDistanceChanged_shouldNotFlyOver": (
                "no_fly_over_max_distance", True),
            "MaxTiltChanged_current": ("max_tilt_changed", True),
            "MaxPitchRollRotationSpeedChanged_current": (
                "max_pitch_roll_rotation_speed_changed", True),
            "MaxVerticalSpeedChanged_current": ("max_vertical_speed_changed",
                                                True),
            "MaxRotationSpeedChanged_current": ("max_rotation_speed_changed",
                                                True),
            "HullProtectionChanged_present": ("hull_protection_changed", True),
            "OutdoorChanged_present": ("outdoor_mode_changed", True),
            "BatteryStateChanged_battery_percent": ("battery", sensor_value),
            "PictureFormatChanged_type": ("picture_format_changed", True),
            "AutoWhiteBalanceChanged_type": ("auto_white_balance_changed",
                                             True),
            "ExpositionChanged_value": ("exposition_changed", True),
            "SaturationChanged_value": ("saturation_changed", True),
            "TimelapseChanged_enabled": ("timelapse_changed", True),
            "VideoStabilizationModeChanged_mode": (
                "video_stabilization_changed", True),
            "VideoRecordingModeChanged_mode": ("video_recording_changed",
                                               True),
            "VideoFramerateChanged_framerate": ("video_framerate_changed",
                                                True),
            "VideoResolutionsChanged_type": ("video_resolutions_changed",
                                             True),
        }

        if sensor_name in state_flags:
            attr_name, attr_value = state_flags[sensor_name]
            if isinstance(attr_value, dict):
                self.CameraMoveEnded.update(attr_value)  # For tilt/pan updates
            else:
                setattr(self, attr_name, attr_value)

    def trigger_user_callback(self) -> None:
        """
        Triggers the user-defined callback function if set, passing the
        specified arguments.
        """
        if self.user_callback_function:
            self.user_callback_function(*self.user_callback_function_args)

    def __str__(self) -> str:
        """
        Returns a string representation of the drone's sensor data.

        :return: Formatted string of sensor data.
        """
        return f"Bebop sensors: {self.sensors_dict}"
