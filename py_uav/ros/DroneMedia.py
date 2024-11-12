"""
DroneMedia: Manages media operations such as recording, video stream handling,
and media state changes.

ROS Topics (4):
    - /bebop/record
    - /bebop/states/ardrone3/MediaStreamingState/VideoEnableChanged
    - /bebop/states/common/MavlinkState/MavlinkFilePlayingStateChanged
    - /bebop/states/common/MavlinkState/MavlinkPlayErrorStateChanged
"""

import rospy
from ..interfaces.RosCommunication import RosCommunication
from std_msgs.msg import Bool, Int32, String


class DroneMedia(RosCommunication):
    """
    Manages media operations on the drone, including video recording,
    snapshot capture, and handling media state changes.
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        """Override __new__ to implement the Singleton pattern."""
        if cls._instance is None:
            cls._instance = super(DroneMedia, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes publishers and subscribers for managing drone media
        operations.

        :param drone_type: The type of drone being used.
        :param frequency: Frequency for media command updates (default: 30 Hz).
        """
        if hasattr(self, '_initialized') and self._initialized:
            return

        super().__init__(drone_type, frequency)
        self.last_command_time = rospy.get_time()

        # Media state variables
        self.video_enabled = False
        self.mavlink_playing_state = 0
        self.mavlink_error = ""

        self._initialize_publishers()
        self._initialize_subscribers()

        self._initialized = True

    def _initialize_subscribers(self) -> None:
        """Sets up subscribers for media state updates."""
        rospy.Subscriber(
            "/bebop/states/ardrone3/MediaStreamingState/VideoEnableChanged",
            Int32, self._video_state_callback
        )
        rospy.Subscriber(
            "/bebop/states/common/MavlinkState/MavlinkFilePlayingStateChanged",
            Int32, self._mavlink_state_callback
        )
        rospy.Subscriber(
            "/bebop/states/common/MavlinkState/MavlinkPlayErrorStateChanged",
            String, self._mavlink_error_callback
        )

    def _initialize_publishers(self) -> None:
        """Initializes publisher for recording operations."""
        self.record_pub = rospy.Publisher("/bebop/record", Bool, queue_size=10)

    def _time_to_update(self) -> bool:
        """
        Checks if enough time has passed to send the next command update.

        :return: True if it's time to update, False otherwise.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    def _video_state_callback(self, msg: Int32) -> None:
        """
        Callback to handle changes in the video streaming state.

        :param msg: Message containing the current video streaming state.
                    (0: disabled, 1: enabled).
        """
        self.video_enabled = bool(msg.data)
        rospy.loginfo(f"Video streaming enabled: {self.video_enabled}")

    def _mavlink_state_callback(self, msg: Int32) -> None:
        """
        Callback to handle changes in MAVLink file playing state.

        :param msg: Message containing the MAVLink file playing state.
        """
        self.mavlink_playing_state = msg.data
        rospy.loginfo(f"MAVLink playing state changed"
                      f": {self.mavlink_playing_state}")

    def _mavlink_error_callback(self, msg: String) -> None:
        """
        Callback to handle MAVLink play errors.

        :param msg: Message containing the MAVLink play error.
        """
        self.mavlink_error = msg.data
        rospy.logwarn(f"MAVLink play error: {self.mavlink_error}")

    def start_recording(self) -> None:
        """Publishes a command to start video recording."""
        rospy.loginfo("Starting video recording.")
        self.record_pub.publish(True)

    def stop_recording(self) -> None:
        """Publishes a command to stop video recording."""
        rospy.loginfo("Stopping video recording.")
        self.record_pub.publish(False)

    def get_video_state(self) -> bool:
        """
        Retrieves the current state of video streaming.

        :return: True if video streaming is enabled, False otherwise.
        """
        return self.video_enabled

    def get_mavlink_playing_state(self) -> int:
        """
        Retrieves the current MAVLink file playing state.

        :return: Integer representing the MAVLink playing state.
        """
        return self.mavlink_playing_state

    def get_mavlink_error(self) -> str:
        """
        Retrieves the latest MAVLink play error message, if any.

        :return: String message with the MAVLink play error.
        """
        return self.mavlink_error
