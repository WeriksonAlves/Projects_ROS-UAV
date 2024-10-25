"""
Purpose: Obtem os dados atraves da leituras dos sensores do drone, incluindo
            GPS, atitude, velocidade e niveis de bateria. Reorganiza os dados
            em um dicionario estruturado.
"""

import time
from ..ros.Sensors import ROSBebop2Sensors


class SensorsParser:
    def __init__(self, drone_type: str, frequence: int = 30):
        """
        Initializes the SensorsParser object.
        """
        self.drone_type = drone_type
        self.period = 1 / frequence
        self.current_time = time.time()

        self.sensors_dict = dict()
        self.RelativeMoveEnded = False
        self.CameraMoveEnded_tilt = False
        self.CameraMoveEnded_pan = False
        self.flying_state = "unknown"
        self.flat_trim_changed = False
        self.max_altitude_changed = False
        self.max_distance_changed = False
        self.no_fly_over_max_distance = False
        self.max_tilt_changed = False
        self.max_pitch_roll_rotation_speed_changed = False
        self.max_vertical_speed_changed = False
        self.max_rotation_speed = False
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

        # Battery and user callback setup
        self.battery = 100
        self.user_callback_function = None
        self.sensors = ROSBebop2Sensors(drone_type)

    # Private Methods

    def _should_process(self) -> bool:
        """Check if the time interval has passed to process."""
        if time.time() - self.current_time > self.period:
            self.current_time = time.time()
            return True
        return False

    def _convert_to_si_units(self, raw_data):
        """
        Converts raw sensor data into SI units.

        :param raw_data: Dictionary of raw sensor data
        :return: Dictionary of data converted to SI units
        """
        si_data = {}

        # Example of unit conversions (dummy logic - replace with actual conversions)
        si_data['speed_mps'] = raw_data.get('speed', 0) * 0.27778  # Convert km/h to m/s
        si_data['altitude_m'] = raw_data.get('altitude', 0)  # Assuming altitude is in meters
        # (Add more conversions as needed for other data points)

        return si_data

    # Callback Methods

    # Public Methods

    def update_sensors(self):
        """Fetches and converts sensor data to SI units at specified intervals."""
        if self._should_process_frame():
            raw_data = self.sensors.get_raw_sensor_data()  # Assuming this method exists
            self.sensors_dict = self._convert_to_si_units(raw_data)
            
            # Call the user callback if it's defined
            if self.user_callback_function:
                self.user_callback_function(self.sensors_dict)

    def get_sensor_data(self):
        """
        Updates the sensors dictionary with the most recent sensor values
        """
        # return Dictionary containing sensor data: odometry, GPS, altitude,
        # attitude, position, speed, flying state, battery level, and WiFi signal.
        self.sensors.update_sensors()




    # Methods to be implemented

    def set_user_callback_function(self, function, args):
        """
        Sets the user callback function (called everytime the sensors are
        updated)

        :param function: name of the user callback function
        :param args: arguments (tuple) to the function
        :return:
        """
        self.user_callback_function = function
        self.user_callback_function_args = args

    def update(self, sensor_name, sensor_value, sensor_enum):
        if (sensor_name is None):
            print("Error empty sensor")
            return

        if (sensor_name, "enum") in sensor_enum:
            # grab the string value
            if (sensor_value is None or sensor_value > len(sensor_enum[
                    (sensor_name, "enum")])):
                value = "UNKNOWN_ENUM_VALUE"
            else:
                enum_value = sensor_enum[(sensor_name, "enum")][sensor_value]
                value = enum_value

            self.sensors_dict[sensor_name] = value

        else:
            # regular sensor
            self.sensors_dict[sensor_name] = sensor_value

        # some sensors are saved outside the dictionary for internal use (they
        # are also in the dictionary)
        if (sensor_name == "FlyingStateChanged_state"):
            self.flying_state = self.sensors_dict["FlyingStateChanged_state"]

        if (sensor_name == "PilotingState_FlatTrimChanged"):
            self.flat_trim_changed = True

        if (sensor_name == "moveByEnd_dX"):
            self.RelativeMoveEnded = True

        if (sensor_name == "OrientationV2_tilt"):
            self.CameraMoveEnded_tilt = True

        if (sensor_name == "OrientationV2_pan"):
            self.CameraMoveEnded_pan = True

        if (sensor_name == "MaxAltitudeChanged_current"):
            self.max_altitude_changed = True

        if (sensor_name == "MaxDistanceChanged_current"):
            self.max_distance_changed = True

        if (sensor_name == "NoFlyOverMaxDistanceChanged_shouldNotFlyOver"):
            self.no_fly_over_max_distance_changed = True

        if (sensor_name == "MaxTiltChanged_current"):
            self.max_tilt_changed = True

        if (sensor_name == "MaxPitchRollRotationSpeedChanged_current"):
            self.max_pitch_roll_rotation_speed_changed = True

        if (sensor_name == "MaxVerticalSpeedChanged_current"):
            self.max_vertical_speed_changed = True

        if (sensor_name == "MaxRotationSpeedChanged_current"):
            self.max_rotation_speed_changed = True

        if (sensor_name == "HullProtectionChanged_present"):
            self.hull_protection_changed = True

        if (sensor_name == "OutdoorChanged_present"):
            self.outdoor_mode_changed = True

        if (sensor_name == "BatteryStateChanged_battery_percent"):
            self.battery = sensor_value

        if (sensor_name == "PictureFormatChanged_type"):
            self.picture_format_changed = True

        if (sensor_name == "AutoWhiteBalanceChanged_type"):
            self.auto_white_balance_changed = True

        if (sensor_name == "ExpositionChanged_value"):
            self.exposition_changed = True

        if (sensor_name == "SaturationChanged_value"):
            self.saturation_changed = True

        if (sensor_name == "TimelapseChanged_enabled"):
            self.timelapse_changed = True

        if (sensor_name == "VideoStabilizationModeChanged_mode"):
            self.video_stabilization_changed = True

        if (sensor_name == "VideoRecordingModeChanged_mode"):
            self.video_recording_changed = True

        if (sensor_name == "VideoFramerateChanged_framerate"):
            self.video_framerate_changed = True

        if (sensor_name == "VideoResolutionsChanged_type"):
            self.video_resolutions_changed = True

        # call the user callback if it isn't None
        if (self.user_callback_function is not None):
            self.user_callback_function(self.user_callback_function_args)

    def __str__(self):
        str = "Bebop sensors: %s" % self.sensors_dict
        return str

    
