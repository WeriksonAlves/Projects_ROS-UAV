import rospy
from .DroneSensorManager import DroneSensorManager
from .DroneCommandManager import DroneCommandManager


class DroneSettingManager:
    """
    Class to manage the settings of the drone, including video, altitude, tilt,
    and more.
    """
    def __init__(self, command_manager: DroneCommandManager,
                 sensor_manager: DroneSensorManager) -> None:
        self.sensors = sensor_manager
        self.command_manager = command_manager

    def _validate_param(self, param, valid_values, param_name):
        if param not in valid_values:
            rospy.logwarn(f"Error: {param} is not a valid {param_name}. Valid"
                          f" values: {valid_values}")
            return False
        return True

    # Settings Functions

    def set_video_stream_mode(self, mode="low_latency"):
        valid_modes = ["low_latency", "high_reliability",
                       "high_reliability_low_framerate"]
        if not self._validate_param(mode, valid_modes, "video stream mode"):
            return False
        self.command_manager.set_video_stream_mode(mode)
        rospy.loginfo("Video stream mode set to: {}".format(mode))

    def set_max_altitude(self, altitude: float):
        if not (0.5 <= altitude <= 150):
            rospy.logwarn(f"Invalid altitude: {altitude}. Must be between 0.5 "
                          "and 150 meters.")
            return
        self.command_manager.set_max_altitude(altitude)
        self.sensors.wait_for_change("max_altitude_changed")

    def set_max_distance(self, distance: float):
        if not (10 <= distance <= 2000):
            rospy.logwarn(f"Invalid distance: {distance}. Must be between 10 "
                          "and 2000 meters.")
            return
        self.command_manager.set_max_distance(distance)
        self.sensors.wait_for_change("max_distance_changed")

    def enable_geofence(self, enable: bool):
        self.command_manager.enable_geofence(int(enable))
        self.sensors.wait_for_change("no_fly_over_max_distance_changed")

    def set_max_tilt(self, tilt: float):
        if not (5 <= tilt <= 30):
            rospy.logwarn(
                f"Invalid tilt: {tilt}. Must be between 5 and 30 degrees.")
            return
        self.command_manager.set_max_tilt(tilt)
        self.sensors.wait_for_change("max_tilt_changed")

    def set_max_tilt_rotation_speed(self, speed: float):
        if not (80 <= speed <= 300):
            rospy.logwarn(f"Invalid tilt rotation speed: {speed}. Must be "
                          "between 80 and 300 degrees per second.")
            return
        self.command_manager.set_max_tilt_rotation_speed(speed)
        self.sensors.wait_for_change("max_pitch_roll_rotation_speed_changed")

    def set_max_vertical_speed(self, speed: float):
        if not (0.5 <= speed <= 2.5):
            rospy.logwarn(f"Invalid vertical speed: {speed}. Must be between "
                          "0.5 and 2.5 m/s.")
            return
        self.command_manager.set_max_vertical_speed(speed)
        self.sensors.wait_for_change("max_vertical_speed_changed")

    def set_max_rotation_speed(self, speed: float):
        if not (10 <= speed <= 200):
            rospy.logwarn(f"Invalid rotation speed: {speed}. Must be between "
                          "10 and 200 degrees per second.")
            return
        self.command_manager.set_max_rotation_speed(speed)
        self.sensors.wait_for_change("max_rotation_speed_changed")

    # Camera and Video Settings

    def set_picture_format(self, format_type: str):
        valid_formats = ['raw', 'jpeg', 'snapshot', 'jpeg_fisheye']
        if not self._validate_param(format_type, valid_formats,
                                    "picture format"):
            return
        self.command_manager.set_picture_format(format_type)
        self.sensors.wait_for_change("picture_format_changed")

    def set_white_balance(self, balance_type: str):
        valid_balances = ['auto', 'tungsten', 'daylight', 'cloudy',
                          'cool_white']
        if not self._validate_param(balance_type, valid_balances,
                                    "white balance type"):
            return
        self.command_manager.set_white_balance(balance_type)
        self.sensors.wait_for_change("auto_white_balance_changed")

    def set_exposition(self, value: float):
        if not (-1.5 <= value <= 1.5):
            rospy.logwarn(f"Invalid exposure: {value}. Must be between -1.5 "
                          "and 1.5.")
            return
        self.command_manager.set_exposition(value)
        self.sensors.wait_for_change("exposition_changed")

    def set_saturation(self, value: int):
        if not (-100 <= value <= 100):
            rospy.logwarn(f"Invalid saturation: {value}. Must be between -100 "
                          "and 100.")
            return
        self.command_manager.set_saturation(value)
        self.sensors.wait_for_change("saturation_changed")

    def set_timelapse(self, enable: bool, interval: int = 8):
        if not (8 <= interval <= 300):
            rospy.logwarn(f"Invalid timelapse interval: {interval}. Must be "
                          "between 8 and 300 seconds.")
            return
        self.command_manager.set_timelapse(int(enable), interval)
        self.sensors.wait_for_change("timelapse_changed")

    def set_video_stabilization(self, mode: str):
        valid_modes = ['roll_pitch', 'pitch', 'roll', 'none']
        if not self._validate_param(mode, valid_modes,
                                    "video stabilization mode"):
            return
        self.command_manager.set_video_stabilization(mode)
        self.sensors.wait_for_change("video_stabilization_changed")

    def set_video_recording(self, mode: str):
        valid_modes = ['quality', 'time']
        if not self._validate_param(mode, valid_modes, "video recording mode"):
            return
        self.command_manager.set_video_recording(mode)
        self.sensors.wait_for_change("video_recording_changed")

    def set_video_framerate(self, framerate: str):
        valid_rates = ['24_FPS', '25_FPS', '30_FPS']
        if not self._validate_param(framerate, valid_rates, "video framerate"):
            return
        self.command_manager.set_video_framerate(framerate)
        self.sensors.wait_for_change("video_framerate_changed")

    def set_video_resolutions(self, resolution: str):
        valid_resolutions = ['rec1080_stream480', 'rec720_stream720']
        if not self._validate_param(resolution, valid_resolutions,
                                    "video resolution"):
            return
        self.command_manager.set_video_resolutions(resolution)
        self.sensors.wait_for_change("video_resolutions_changed")
