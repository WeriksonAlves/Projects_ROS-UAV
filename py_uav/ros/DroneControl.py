"""
Purpose: This class handles the basic control of the Bebop drone, including
taking off, landing, navigation, and velocity control.

Topics (10):
    /bebop/takeoff
    /bebop/land
    /bebop/cmd_vel
    /bebop/reset
    /bebop/flattrim
    /bebop/flip
    /bebop/autoflight/navigate_home
    /bebop/autoflight/pause
    /bebop/autoflight/start
    /bebop/autoflight/stop
"""

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8, Bool


class DroneControl:
    """
    DroneControl manages basic control operations of the Bebop drone,
    including takeoff, landing, velocity control, reset, flat trim, flips,
    and autopilot commands via ROS topics.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize the control class for the drone and set up ROS publishers.

        :param drone_type: Specifies the drone type, allowing for custom
                           configurations.
        :param frequency: Control command frequency in Hz (default is 30 Hz).
        """
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = time.time()
        self.vel_cmd = Twist()

        try:
            self.publishers = self._initialize_publishers()
            rospy.loginfo(f"DroneControl initialized for {self.drone_type}.")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to initialize DroneControl: {e}")
            quit()

    def _initialize_publishers(self) -> dict:
        """
        Set up and return a dictionary of ROS publishers for controlling the
        drone.

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
        Publish a command to its corresponding ROS topic.

        :param command: Command name (e.g., 'takeoff', 'land').
        :param message: Message to publish; defaults to Empty if None.
        """
        publisher = self.publishers.get(command)
        if publisher:
            message = message if message else Empty()
            publisher.publish(message)
            rospy.loginfo(f"Published {command} command.")
        else:
            rospy.logwarn(f"Command {command} not recognized.")

    def _is_time_to_command(self) -> bool:
        """
        Check if enough time has passed since the last command.

        :return: True if the command interval has passed; otherwise, False.
        """
        current_time = time.time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    # Core drone control methods

    def takeoff(self) -> None:
        """Send the takeoff command to the drone."""
        self._publish_command('takeoff')

    def land(self) -> None:
        """Send the landing command to the drone."""
        self._publish_command('land')

    def reset(self) -> None:
        """Send the reset command to the drone."""
        self._publish_command('reset')

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             linear_z: float = 0.0, angular_z: float = 0.0) -> None:
        """
        Control the drone's movement by setting velocities along each axis.

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
        """Send the flat trim calibration command to the drone."""
        self._publish_command('flattrim')

    def flip(self, direction: str) -> None:
        """
        Command the drone to perform a flip in a specified direction.

        :param direction: Direction for the flip ('forward', 'backward',
                          'left', 'right').
        """
        flip_directions = {
            'forward': UInt8(0),
            'backward': UInt8(1),
            'left': UInt8(2),
            'right': UInt8(3)
        }
        if direction in flip_directions:
            self._publish_command('flip', flip_directions[direction])
        else:
            rospy.logwarn(f"Invalid flip direction: {direction}")

    # Autopilot control methods

    def navigate_home(self, start: bool) -> None:
        """
        Command the drone to start or stop navigation to home.

        :param start: True to start navigating home, False to stop.
        """
        self._publish_command('navigate_home', Bool(data=start))

    def pause(self) -> None:
        """Pause any ongoing autopilot mission."""
        self._publish_command('pause')

    def start_autoflight(self) -> None:
        """Start an autopilot mission."""
        self._publish_command('start')

    def stop_autoflight(self) -> None:
        """Stop the current autopilot mission."""
        self._publish_command('stop')
