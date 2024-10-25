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


class ROSBebop2Control:
    """
    DroneControl manages the core operations of the Bebop drone, such as
    takeoff, landing, movement, reset, flat trim, flips, and autopilot
    commands via ROS topics.
    """

    def __init__(self, drone_type: str, frequence: int = 30):
        """
        Initialize the DroneControl class and set up ROS publishers for drone
        commands.

        :param drone_type: Type of the drone (for future use, e.g., different
        command sets).
        :param frequence: Time interval between commands (default: 30 Hz).
        """
        self.drone_type = drone_type
        self.period = 1 / frequence
        self.current_time = time.time()

        self.vel_cmd = Twist()
        self.pubs = {}

        self._initialize_publishers()
        rospy.loginfo(f"DroneControl initialized for {self.drone_type}.")

    def _initialize_publishers(self) -> None:
        """
        Initialize all necessary ROS publishers for controlling the drone.
        """
        topics = {
            'takeoff': '/bebop/takeoff',
            'land': '/bebop/land',
            'reset': '/bebop/reset',
            'cmd_vel': '/bebop/cmd_vel',
            'flattrim': '/bebop/flattrim',
            'flip': '/bebop/flip',
            'navigate_home': '/bebop/autoflight/navigate_home',
            'pause': '/bebop/autoflight/pause',
            'start': '/bebop/autoflight/start',
            'stop': '/bebop/autoflight/stop'
        }

        for key, topic in topics.items():
            # Adjust message type based on the topic
            if key in ['cmd_vel']:
                self.pubs[key] = rospy.Publisher(topic, Twist, queue_size=10)
            elif key in ['navigate_home']:
                self.pubs[key] = rospy.Publisher(topic, Bool, queue_size=10)
            else:
                self.pubs[key] = rospy.Publisher(topic, Empty, queue_size=10)

        rospy.loginfo("All ROS publishers initialized for drone control.")

    def _publish_command(self, command: str, message=None) -> None:
        """
        Publish a command to the corresponding ROS topic.

        :param command: The name of the command (e.g., 'takeoff', 'land').
        :param message: The message to be published (default: Empty message).
        """
        if command in self.pubs:
            if message is None:
                message = Empty()  # Default message is Empty if not provided
            self.pubs[command].publish(message)
            rospy.loginfo(f"Published {command} command.")
        else:
            rospy.logwarn(f"Command {command} not found.")

    def _should_process_frame(self) -> bool:
        """Check if the time interval has passed to process the next frame."""
        if time.time() - self.current_time > (self.period):
            self.current_time = time.time()
            return True
        return False

    # Drone control methods

    def takeoff(self) -> None:
        """Command the drone to take off."""
        self._publish_command('takeoff')

    def land(self) -> None:
        """Command the drone to land."""
        self._publish_command('land')

    def reset(self) -> None:
        """Command the drone to reset."""
        self._publish_command('reset')

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             linear_z: float = 0.0, angular_z: float = 0.0) -> None:
        """
        Command the drone to move based on velocity inputs.

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
        """Command the drone to perform a flat trim calibration."""
        self._publish_command('flattrim')

    def flip(self, direction: str) -> None:
        """Command the drone to flip in a specified direction."""
        flip_map = {'forward': UInt8(0), 'backward': UInt8(1),
                    'left': UInt8(2), 'right': UInt8(3)}
        if direction in flip_map:
            self._publish_command('flip', flip_map[direction])
        else:
            rospy.logwarn(f"Invalid flip direction: {direction}")

    # Autopilot control methods

    def navigate_home(self, start: bool) -> None:
        """
        Command the drone to navigate to home.

        :param start: True to start navigating home, False to stop.
        """
        self._publish_command('navigate_home', Bool(data=start))

    def pause(self) -> None:
        """Command the drone to pause an ongoing autopilot mission."""
        self._publish_command('pause')

    def start_autoflight(self) -> None:
        """Command the drone to start an autopilot mission."""
        self._publish_command('start')

    def stop_autoflight(self) -> None:
        """Command the drone to stop an autopilot mission."""
        self._publish_command('stop')
