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
            rospy.logwarn("Move command ignored: Inputs must be between -1 and 1.")
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
