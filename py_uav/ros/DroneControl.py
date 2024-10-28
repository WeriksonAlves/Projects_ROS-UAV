"""
Purpose: Manages the Bebop drone's basic control operations, including
takeoff, landing, movement, flips, and autopilot commands through ROS topics.

Used Topics (10):
    - /bebop/takeoff
    - /bebop/land
    - /bebop/cmd_vel
    - /bebop/reset
    - /bebop/flattrim
    - /bebop/flip
    - /bebop/autoflight/navigate_home
    - /bebop/autoflight/pause
    - /bebop/autoflight/start
    - /bebop/autoflight/stop
"""

import rospy
import time
from ..interfaces.RosCommunication import RosCommunication
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8, Bool


class DroneControl(RosCommunication):
    """
    Manages basic control operations for the Bebop drone, such as takeoff,
    landing, movement, and autopilot controls via ROS publishers.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes the DroneControl class with required ROS publishers
        for drone commands.

        :param drone_type: Specifies the type of drone (e.g., "Bebop2").
        :param frequency: Command frequency in Hz (default: 30).
        """
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = time.time()
        self.vel_cmd = Twist()
        try:
            self.publishers = self._initialize_publishers()
            rospy.loginfo(f"DroneControl initialized for {self.drone_type}.")
        except rospy.ROSException as e:
            rospy.logerr(f"DroneControl initialization failed: {e}")
            quit()

    def _initialize_subscribers(self) -> None:
        """Sets up ROS subscribers; currently no subscribers are required."""
        pass

    def _initialize_publishers(self) -> dict:
        """
        Initializes and returns a dictionary of ROS publishers for drone
        commands.

        :return: Dictionary mapping command names to their ROS publishers.
        """
        topics = {
            'takeoff': ('/bebop/takeoff', Empty),
            'land': ('/bebop/land', Empty),
            'reset': ('/bebop/reset', Empty),
            'cmd_vel': ('/bebop/cmd_vel', Twist),
            'flattrim': ('/bebop/flattrim', Empty),
            'flip': ('/bebop/flip', UInt8),
            'navigate_home': ('/bebop/autoflight/navigate_home', Bool),
            'pause': ('/bebop/autoflight/pause', Empty),
            'start': ('/bebop/autoflight/start', Empty),
            'stop': ('/bebop/autoflight/stop', Empty),
        }
        return {
            name: rospy.Publisher(topic, msg_type, queue_size=10)
            for name, (topic, msg_type) in topics.items()
        }

    def _publish_command(self, command: str, message=None) -> None:
        """
        Publishes a command to its ROS topic.

        :param command: Command name to publish (e.g., 'takeoff', 'land').
        :param message: Message to publish; defaults to Empty() if None.
        """
        publisher = self.publishers.get(command)
        if publisher:
            publisher.publish(message or Empty())
            rospy.loginfo(f"Published {command} command.")
        else:
            rospy.logwarn(f"Unknown command: {command}")

    def _is_time_to_command(self) -> bool:
        """
        Checks if enough time has passed since the last command.

        :return: True if the command interval has passed; False otherwise.
        """
        current_time = time.time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    # Core control methods

    def takeoff(self) -> None:
        """Commands the drone to take off."""
        self._publish_command('takeoff')

    def land(self) -> None:
        """Commands the drone to land."""
        self._publish_command('land')

    def reset(self) -> None:
        """Commands the drone to reset."""
        self._publish_command('reset')

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             linear_z: float = 0.0, angular_z: float = 0.0) -> None:
        """
        Commands the drone to move with specified velocities.

        :param linear_x: Forward/backward velocity.
        :param linear_y: Left/right velocity.
        :param linear_z: Up/down velocity.
        :param angular_z: Rotational velocity around the Z-axis (yaw).
        """
        self.vel_cmd.linear.x = linear_x
        self.vel_cmd.linear.y = linear_y
        self.vel_cmd.linear.z = linear_z
        self.vel_cmd.angular.z = angular_z
        self._publish_command('cmd_vel', self.vel_cmd)

    def flattrim(self) -> None:
        """Commands the drone to perform flat trim calibration."""
        self._publish_command('flattrim')

    def flip(self, direction: str) -> None:
        """
        Commands the drone to perform a flip in the specified direction.

        :param direction: Direction for flip ('forward', 'backward', 'left',
                          or 'right').
        """
        flip_directions = {
            'forward': UInt8(0),
            'backward': UInt8(1),
            'left': UInt8(2),
            'right': UInt8(3)
        }
        flip_cmd = flip_directions.get(direction)
        if flip_cmd is not None:
            self._publish_command('flip', flip_cmd)
        else:
            rospy.logwarn(f"Invalid flip direction: {direction}")

    # Autopilot control methods

    def navigate_home(self, start: bool) -> None:
        """
        Commands the drone to start or stop navigation to home.

        :param start: True to start navigating home, False to stop.
        """
        self._publish_command('navigate_home', Bool(data=start))

    def pause(self) -> None:
        """Pauses any ongoing autopilot mission."""
        self._publish_command('pause')

    def start_autoflight(self) -> None:
        """Starts an autopilot mission."""
        self._publish_command('start')

    def stop_autoflight(self) -> None:
        """Stops the current autopilot mission."""
        self._publish_command('stop')
