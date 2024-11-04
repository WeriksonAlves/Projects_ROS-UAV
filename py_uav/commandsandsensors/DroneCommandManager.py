import rospy
from ..ros.DroneControl import DroneControl
from typing import Callable


class DroneCommandManager:
    """
    Manages drone commands, verifying the state of the drone and ensuring
    commands are executed only when appropriate.
    """

    def __init__(self, control: DroneControl, sensors: object,
                 state_map: dict):
        """
        Initializes the DroneCommandManager with control, sensor, and state
        mappings.

        :param control: DroneControl instance for issuing commands.
        :param sensors: Sensors instance providing current drone state.
        :param state_map: Mapping of state keys to state descriptions.
        """
        self.control = control
        self.sensors = sensors
        self.state_map = state_map

    def _is_state(self, state_key: str) -> bool:
        """
        Checks if the drone's current state matches the specified state.

        :param state_key: Key representing the desired state.
        :return: True if the current state matches; False otherwise.
        """
        return self.sensors.flying_state == self.state_map.get(state_key)

    def _execute_command(self, command_func: Callable, *args) -> None:
        """
        Executes a command if the drone is not in an emergency state.

        :param command_func: Command function to be executed.
        :param args: Arguments for the command function.
        """
        if not self._is_state('E'):
            command_func(*args)
        else:
            rospy.logwarn("Command aborted: Drone is in emergency mode.")

    def takeoff(self) -> None:
        """
        Commands the drone to take off if it is in a landed state.

        :return: None
        """
        if self._is_state('L'):
            self._execute_command(self.control.takeoff)
        else:
            rospy.loginfo(
                "Takeoff command ignored: Drone is not in landed state.")

    def land(self) -> None:
        """
        Commands the drone to land if it is in a hovering state.

        :return: None
        """
        if self._is_state('H'):
            self._execute_command(self.control.land)
        else:
            rospy.loginfo(
                "Land command ignored: Drone is not in hovering state.")

    def move(self, roll: float, pitch: float, yaw: float, vertical: float
             ) -> None:
        """
        Moves the drone with specified directional inputs if it is hovering.

        :param roll: Roll input (-1 to 1).
        :param pitch: Pitch input (-1 to 1).
        :param yaw: Yaw input (-1 to 1).
        :param vertical: Vertical input (-1 to 1).
        :return: None
        """
        if self._is_state('H'):
            self._execute_command(self.control.move, roll, pitch, yaw,
                                  vertical)
        else:
            rospy.loginfo(
                "Move command ignored: Drone is not in hovering state.")

    def flip(self, direction: str) -> None:
        """
        Commands the drone to perform a flip in a specified direction if it is
        hovering.

        :param direction: Direction for the flip ('forward', 'backward',
        'left', 'right').
        :return: None
        """
        if direction.lower() not in ['forward', 'backward', 'left', 'right']:
            rospy.logwarn("Invalid flip direction. Choose 'forward', "
                          "'backward', 'left', or 'right'.")
            return

        if self._is_state('H'):
            self._execute_command(self.control.flip, direction)
        else:
            rospy.loginfo(
                "Flip command ignored: Drone is not in hovering state.")
