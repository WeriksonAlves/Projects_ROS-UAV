"""
Purpose: This class manages media operations such as recording video streams,
handling media state changes, and snapshot capture.

Topics (4):
    /bebop/record
    /bebop/states/ardrone3/MediaStreamingState/VideoEnableChanged
    /bebop/states/common/MavlinkState/MavlinkFilePlayingStateChanged
    /bebop/states/common/MavlinkState/MavlinkPlayErrorStateChanged
"""

import rospy
import time
from ..interfaces.RosCommunication import RosCommunication
from std_msgs.msg import Bool, Int32, String


class DroneMedia(RosCommunication):
    """
    Class for managing media operations on the drone, such as video recording,
    snapshot capture, and handling media state changes.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes publishers and subscribers for managing drone media
        operations.

        :param drone_type: The type of drone being used.
        :param frequency: The frequency at which to send media commands.
        """
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = time.time()

        self._initialize_publishers()
        self._initialize_subscribers()

        # Variable to hold the current media states
        self.video_enabled = None
        self.mavlink_playing_state = None
        self.mavlink_error = None

    def _initialize_subscribers(self) -> None:
        """
        Initializes subscribers for media state updates.
        """
        rospy.Subscriber(
            "/bebop/states/ardrone3/MediaStreamingState/VideoEnableChanged",
            Int32, self._video_state_callback)
        rospy.Subscriber(
            "/bebop/states/common/MavlinkState/MavlinkFilePlayingStateChanged",
            Int32, self._mavlink_state_callback)
        rospy.Subscriber(
            "/bebop/states/common/MavlinkState/MavlinkPlayErrorStateChanged",
            String, self._mavlink_error_callback)

    def _initialize_publishers(self) -> None:
        """
        Initializes publishers for media operations.
        """
        self.record_pub = rospy.Publisher("/bebop/record", Bool, queue_size=10)

    def _time_to_update(self) -> bool:
        """
        Check if enough time has passed for the next camera operation update.
        """
        if time.time() - self.last_update_time >= self.command_interval:
            self.last_update_time = time.time()
            return True
        return False

    def _video_state_callback(self, msg) -> None:
        """
        Callback function for the video enable state.
        Updates the state of video streaming.

        :param msg: The current state of video streaming (0: disabled,
                    1: enabled).
        """
        self.video_enabled = bool(msg.data)
        rospy.loginfo(f"Video streaming enabled: {self.video_enabled}")

    def _mavlink_state_callback(self, msg) -> None:
        """
        Callback function for the MAVLink playing state.
        Updates the MAVLink file playing state.

        :param msg: The current MAVLink playing state.
        """
        self.mavlink_playing_state = msg.data
        rospy.loginfo(f"MAVLink playing state changed"
                      f": {self.mavlink_playing_state}")

    def _mavlink_error_callback(self, msg) -> None:
        """
        Callback function for MAVLink play error state.
        Updates the MAVLink play error state.

        :param msg: The MAVLink play error message.
        """
        self.mavlink_error = msg.data
        rospy.logwarn(f"MAVLink play error: {self.mavlink_error}")

    def start_recording(self) -> None:
        """
        Sends a message to start video recording.
        """
        rospy.loginfo("Starting video recording.")
        self.record_pub.publish(True)

    def stop_recording(self) -> None:
        """
        Sends a message to stop video recording.
        """
        rospy.loginfo("Stopping video recording.")
        self.record_pub.publish(False)

    def get_video_state(self) -> bool:
        """
        Returns the current video streaming state.

        :return: Current video streaming state.
        """
        return self.video_enabled

    def get_mavlink_playing_state(self) -> int:
        """
        Returns the current MAVLink file playing state.

        :return: Current MAVLink file playing state.
        """
        return self.mavlink_playing_state

    def get_mavlink_error(self) -> str:
        """
        Returns the current MAVLink play error message.

        :return: Current MAVLink play error message.
        """
        return self.mavlink_error
