import rospy
from typing import Callable
from .DroneSensorManager import DroneSensorManager
from ..ros.DroneCamera import DroneCamera
from ..ros.DroneControl import DroneControl


class DroneCommandManager:
    """
    Manages drone commands, verifying the drone's state to ensure commands
    are executed only when appropriate.
    """

    def __init__(self, sensors: DroneSensorManager, state_map: dict,
                 drone_type: str, frequency: int = 30):
        """
        Initializes the DroneCommandManager with control, sensor, and state
        mappings.

        :param sensors: Instance providing current drone state.
        :param state_map: Mapping of state keys to state descriptions.
        :param drone_type: Type of the drone.
        :param frequency: Command update frequency in Hz (default: 30 Hz).
        """
        self.camera = self._initialize_camera(drone_type, frequency)
        self.control = self._initialize_control(drone_type, frequency)
        self.sensors = sensors
        self.state_map = state_map
        self.command_interval = 1 / frequency
        self.last_command_time = rospy.get_time()

    @staticmethod
    def _initialize_camera(drone_type: str, frequency: int) -> DroneCamera:
        """Initializes the camera for the specified drone type."""
        return DroneCamera(drone_type, frequency)

    @staticmethod
    def _initialize_control(drone_type: str, frequency: int) -> DroneControl:
        """Initializes the control for the specified drone type."""
        return DroneControl(drone_type, frequency)

    def _is_in_state(self, state_key: str) -> bool:
        """
        Checks if the drone's current state matches the specified state.

        :param state_key: Key representing the desired state.
        :return: True if the current state matches; False otherwise.
        """
        return self.sensors.get_sensor_data('flying_state'
                                            ) == self.state_map.get(state_key)

    def _execute_if_allowed(self, command_func: Callable, *args) -> None:
        """
        Executes a command if the drone is not in an emergency state and
        adheres to the command interval.

        :param command_func: Command function to be executed.
        :param args: Arguments for the command function.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time < self.command_interval:
            rospy.logwarn("Command ignored: Command interval limit reached.")
            return

        self.last_command_time = current_time

        if not self._is_in_state('E'):
            command_func(*args)
        else:
            rospy.logwarn("Command aborted: Drone is in emergency mode.")

    # Drone Control Commands

    def takeoff(self) -> None:
        """Commands the drone to take off if it is in a landed state."""
        if self._is_in_state('L'):
            self._execute_if_allowed(self.control.takeoff)
        else:
            rospy.loginfo("Takeoff ignored: Drone is not in landed state.")

    def land(self) -> None:
        """Commands the drone to land if it is in a hovering state."""
        if self._is_in_state('H'):
            self._execute_if_allowed(self.control.land)
        else:
            rospy.loginfo("Land ignored: Drone is not in hovering state.")

    def move(self, roll: float, pitch: float, yaw: float, vertical: float
             ) -> None:
        """
        Moves the drone with specified directional inputs if it is hovering.

        :param roll: Roll input (-1 to 1).
        :param pitch: Pitch input (-1 to 1).
        :param yaw: Yaw input (-1 to 1).
        :param vertical: Vertical input (-1 to 1).
        """
        if not all(-1 <= param <= 1 for param in [roll, pitch, yaw, vertical]):
            rospy.logwarn(
                "Move command ignored: Inputs must be between -1 and 1.")
            return
        if self._is_in_state('H'):
            self._execute_if_allowed(self.control.move, roll, pitch, yaw,
                                     vertical)
        else:
            rospy.loginfo("Move ignored: Drone is not in hovering state.")

    def flip(self, direction: str) -> None:
        """
        Commands the drone to perform a flip in a specified direction if it is
        hovering.

        :param direction: Direction for the flip ('forward', 'backward',
                          'left', 'right').
        """
        valid_directions = {'forward', 'backward', 'left', 'right'}
        if direction.lower() not in valid_directions:
            rospy.logwarn(f"Invalid flip direction '{direction}'. Choose from"
                          f" {valid_directions}.")
            return
        if self._is_in_state('H'):
            self._execute_if_allowed(self.control.flip, direction)
        else:
            rospy.loginfo("Flip ignored: Drone is not in hovering state.")

    # Camera Commands

    def adjust_camera_orientation(self, tilt: float, pan: float) -> None:
        """
        Adjusts the camera tilt and pan if the drone is not in an emergency
        state.

        :param tilt: Tilt adjustment (-1 to 1).
        :param pan: Pan adjustment (-1 to 1).
        """
        if not (-1 <= tilt <= 1 and -1 <= pan <= 1):
            rospy.logwarn("Camera orientation adjustment ignored: Tilt and pan"
                          " must be between -1 and 1.")
            return
        if not self._is_in_state('E'):
            self.camera.control_camera_orientation(tilt, pan)
        else:
            rospy.logwarn(
                "Camera adjustment aborted: Drone is in emergency mode.")

    def take_picture(self) -> None:
        """Captures a picture with the drone's camera."""
        if not self._is_in_state('E'):
            self._execute_if_allowed(self.camera.capture_snapshot)
        else:
            rospy.logwarn("Capture aborted: Drone is in emergency mode.")

    def set_video_stream_mode(self, mode: str) -> None:
        """
        Sets the video stream mode.
        :param mode: Video stream mode.
        """
        # Assume there's a ROS service or command for setting video stream
        # mode.
        rospy.loginfo(f"Setting video stream mode to {mode}")
        # Implement the actual ROS service or command call here

    def set_max_altitude(self, altitude: float) -> None:
        """
        Sets the maximum altitude for the drone.
        :param altitude: Maximum altitude in meters.
        """
        rospy.loginfo(f"Setting max altitude to {altitude} meters")
        # Implement the actual ROS service or command call here

    def set_max_distance(self, distance: float) -> None:
        """
        Sets the maximum flight distance.
        :param distance: Maximum distance in meters.
        """
        rospy.loginfo(f"Setting max distance to {distance} meters")
        # Implement the actual ROS service or command call here

    def enable_geofence(self, enable: int) -> None:
        """
        Enables or disables the geofence.
        :param enable: 1 to enable, 0 to disable.
        """
        rospy.loginfo(
            f"Setting geofence to {'enabled' if enable else 'disabled'}")
        # Implement the actual ROS service or command call here

    def set_max_tilt(self, tilt: float) -> None:
        """
        Sets the maximum tilt for the drone.
        :param tilt: Maximum tilt in degrees.
        """
        rospy.loginfo(f"Setting max tilt to {tilt} degrees")
        # Implement the actual ROS service or command call here

    def set_max_tilt_rotation_speed(self, speed: float) -> None:
        """
        Sets the maximum tilt rotation speed.
        :param speed: Tilt rotation speed in degrees per second.
        """
        rospy.loginfo(
            f"Setting max tilt rotation speed to {speed} degrees/second")
        # Implement the actual ROS service or command call here

    def set_max_vertical_speed(self, speed: float) -> None:
        """
        Sets the maximum vertical speed.
        :param speed: Vertical speed in meters per second.
        """
        rospy.loginfo(f"Setting max vertical speed to {speed} m/s")
        # Implement the actual ROS service or command call here

    def set_max_rotation_speed(self, speed: float) -> None:
        """
        Sets the maximum rotation speed.
        :param speed: Rotation speed in degrees per second.
        """
        rospy.loginfo(f"Setting max rotation speed to {speed} degrees/second")
        # Implement the actual ROS service or command call here

    def set_picture_format(self, format_type: str) -> None:
        """
        Sets the picture format.
        :param format_type: Picture format.
        """
        rospy.loginfo(f"Setting picture format to {format_type}")
        # Implement the actual ROS service or command call here

    def set_white_balance(self, balance_type: str) -> None:
        """
        Sets the white balance.
        :param balance_type: White balance type.
        """
        rospy.loginfo(f"Setting white balance to {balance_type}")
        # Implement the actual ROS service or command call here

    def set_exposition(self, value: float) -> None:
        """
        Sets the exposition level.
        :param value: Exposition adjustment.
        """
        rospy.loginfo(f"Setting exposition to {value}")
        # Implement the actual ROS service or command call here

    def set_saturation(self, value: int) -> None:
        """
        Sets the saturation level.
        :param value: Saturation adjustment.
        """
        rospy.loginfo(f"Setting saturation to {value}")
        # Implement the actual ROS service or command call here

    def set_timelapse(self, enable: int, interval: int) -> None:
        """
        Enables or disables timelapse and sets interval.
        :param enable: 1 to enable, 0 to disable.
        :param interval: Interval in seconds.
        """
        rospy.loginfo(
            f"Setting timelapse to {'enabled' if enable else 'disabled'}, "
            f"interval {interval} seconds")
        # Implement the actual ROS service or command call here

    def set_video_stabilization(self, mode: str) -> None:
        """
        Sets video stabilization mode.
        :param mode: Stabilization mode.
        """
        rospy.loginfo(f"Setting video stabilization mode to {mode}")
        # Implement the actual ROS service or command call here

    def set_video_recording(self, mode: str) -> None:
        """
        Sets video recording mode.
        :param mode: Recording mode.
        """
        rospy.loginfo(f"Setting video recording mode to {mode}")
        # Implement the actual ROS service or command call here

    def set_video_framerate(self, framerate: str) -> None:
        """
        Sets the video framerate.
        :param framerate: Framerate setting.
        """
        rospy.loginfo(f"Setting video framerate to {framerate}")
        # Implement the actual ROS service or command call here

    def set_video_resolutions(self, resolution: str) -> None:
        """
        Sets the video resolution.
        :param resolution: Resolution setting.
        """
        rospy.loginfo(f"Setting video resolution to {resolution}")
        # Implement the actual ROS service or command call here
