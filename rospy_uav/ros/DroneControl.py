"""
DroneControl: Manages the Bebop drone's basic control operations, including
takeoff, landing, movement, flips, and autopilot commands through ROS topics.
"""

import rospy
from ..interfaces.RosCommunication import RosCommunication
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8, Bool, String


class DroneControl(RosCommunication):
    """
    Manages basic control operations for the Bebop drone, including takeoff,
    landing, movement, flips, and autopilot controls via ROS publishers.
    """

    _instance = None  # Singleton instance

    def __new__(cls, *args, **kwargs):
        """Implement the Singleton pattern to ensure only one instance."""
        if cls._instance is None:
            cls._instance = super(DroneControl, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes the DroneControl class with ROS publishers for drone
        commands.

        :param drone_type: The type of the drone.
        :param frequency: Command frequency in Hz (default: 30).
        """
        if getattr(self, '_initialized', False):
            return  # Prevent reinitialization in Singleton pattern

        super().__init__(drone_type, frequency)  # self.command_interval
        self.last_command_time = rospy.get_time()
        self.vel_cmd = Twist()

        # Set up ROS publishers and parameters
        self.publishers = self._initialize_publishers()

        self._initialized = True
        rospy.loginfo(f"DroneControl initialized for {self.drone_type}.")

    def _initialize_subscribers(self) -> None:
        """Sets up ROS subscribers; currently no subscribers are required."""
        return super()._initialize_subscribers()

    def _initialize_publishers(self) -> dict:
        """
        Initializes ROS publishers for drone commands.

        :return: Dictionary of ROS publishers for drone commands.
        """
        topics = {
            'bebop2': {
                'takeoff': ('/bebop/takeoff', Empty),
                'land': ('/bebop/land', Empty),
                'reset': ('/bebop/reset', Empty),
                'cmd_vel': ('/bebop/cmd_vel', Twist),
                'flattrim': ('/bebop/flattrim', Empty),
                'flip': ('/bebop/flip', UInt8),
                'navigate_home': ('/bebop/autoflight/navigate_home', Bool),
                'pause': ('/bebop/autoflight/pause', Empty),
                'start': ('/bebop/autoflight/start', String),
                'stop': ('/bebop/autoflight/stop', Empty),
            },
            'gazebo': {
                'takeoff': ('/bebop/takeoff', Empty),
                'land': ('/bebop/land', Empty),
                'reset': ('/bebop/reset', Empty),
                'cmd_vel': ('/bebop/cmd_vel', Twist),
            },
        }.get(self.drone_type.lower(), {})

        if not topics:
            rospy.logwarn(f"Unknown drone type: {self.drone_type}")
        return {
            name: rospy.Publisher(topic, msg_type, queue_size=10)
            for name, (topic, msg_type) in topics.items()
        }

    def _publish_command(self, command: str, message=None) -> None:
        """
        Publishes a command to its corresponding ROS topic.

        :param command: Command name (e.g., 'takeoff', 'land').
        :param message: ROS message to publish (default: Empty()).
        """
        publisher: rospy.Publisher = self.publishers.get(command)
        if publisher:
            publisher.publish(message or Empty())
            rospy.loginfo(f"Published '{command}' command.")
        else:
            rospy.logwarn(f"Command '{command}' is not available for this"
                          " drone type.")

    def _is_time_to_command(self) -> bool:
        """
        Checks if the minimum interval between commands has elapsed.

        :return: True if sufficient time has passed, False otherwise.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    # Core Control Methods

    def takeoff(self) -> None:
        """Commands the drone to take off."""
        if self._is_time_to_command():
            self._publish_command('takeoff')

    def land(self) -> None:
        """Commands the drone to land."""
        if self._is_time_to_command():
            self._publish_command('land')

    def reset(self) -> None:
        """Commands the drone to reset."""
        if self._is_time_to_command():
            self._publish_command('reset')

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             linear_z: float = 0.0, angular_z: float = 0.0) -> None:
        """
        Commands the drone to move with the specified velocities.

        :param linear_x: Forward/backward velocity (m/s).
        :param linear_y: Left/right velocity (m/s).
        :param linear_z: Up/down velocity (m/s).
        :param angular_z: Rotational velocity around the Z-axis (rad/s).
        """
        self.vel_cmd.linear.x = linear_x
        self.vel_cmd.linear.y = linear_y
        self.vel_cmd.linear.z = linear_z
        self.vel_cmd.angular.z = angular_z
        # self._publish_command('cmd_vel', self.vel_cmd)

    def flattrim(self) -> None:
        """Commands the drone to perform a flat trim calibration."""
        if self._is_time_to_command():
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
            'right': UInt8(3),
        }
        flip_cmd = flip_directions.get(direction)
        if flip_cmd:
            self._publish_command('flip', flip_cmd)
        else:
            rospy.logwarn(f"Invalid flip direction: {direction}")

    # Autopilot Control Methods

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
