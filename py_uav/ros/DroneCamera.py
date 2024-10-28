"""
Purpose: This class handles camera operations for the Bebop drone, including
capturing raw images, managing camera orientation, and controlling exposure
settings.

Topics (10):
    /bebop/image_raw
    /bebop/image_raw/compressed
    /bebop/image_raw/compressed/parameter_descriptions
    /bebop/image_raw/compressed/parameter_updates
    /bebop/image_raw/compressedDepth
    /bebop/image_raw/theora
    /bebop/camera_control
    /bebop/states/ardrone3/CameraState/Orientation
    /bebop/set_exposure
    /bebop/snapshot

Missing topics (5):
    /bebop/camera_info
    /bebop/image_raw/compressedDepth/parameter_descriptions
    /bebop/image_raw/compressedDepth/parameter_updates
    /bebop/image_raw/theora/parameter_descriptions
    /bebop/image_raw/theora/parameter_updates
"""

import cv2
import numpy as np
import rospy
import time
from ..interfaces.RosCommunication import RosCommunication
from bebop_msgs.msg import Ardrone3CameraStateOrientation
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import ConfigDescription, Config
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty, Float32
from typing import List


class CameraControl(RosCommunication):
    """
    Manages camera operations for the Bebop drone, including capturing images,
    managing camera orientation, and controlling exposure settings through ROS
    topics.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize the CameraControl class with required publishers,
        subscribers, and image handling configurations.

        :param drone_type: Type of the drone (e.g., "Bebop2").
        :param frequency: Frequency of camera operations (default: 30 Hz).
        """
        self.drone_type = drone_type
        self.period = 1 / frequency
        self.last_update_time = time.time()
        self.image_data = {key: None for key in ['image', 'compressed',
                                                 'depth', 'theora']}
        self.success_flags = {key: False for key in self.image_data.keys()}
        self.bridge = CvBridge()
        self.orientation = {'tilt': 0.0, 'pan': 0.0}
        self.param_listener = ParameterListener(self)

        try:
            self.pubs = self._initialize_publishers()
            self.subs = self._initialize_subscribers()
            rospy.loginfo(f"CameraControl initialized for {self.drone_type}.")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to initialize CameraControl: {e}")
            quit()

    def _initialize_publishers(self) -> dict:
        """Initialize ROS publishers for camera commands."""
        try:
            topics = {
                'camera_control': rospy.Publisher(
                    '/bebop/camera_control', Twist, queue_size=10),
                'snapshot': rospy.Publisher(
                    '/bebop/snapshot', Empty, queue_size=10),
                'set_exposure': rospy.Publisher(
                    '/bebop/set_exposure', Float32, queue_size=10)
            }
            return topics
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to initialize publishers: {e}")
            quit()

    def _initialize_subscribers(self) -> dict:
        """Initialize ROS subscribers for camera data and orientation."""
        topics = {
            'image': rospy.Subscriber(
                "/bebop/image_raw", Image, self._process_raw_image),
            'compressed': rospy.Subscriber(
                "/bebop/image_raw/compressed", CompressedImage,
                self._process_compressed_image),
            'depth': rospy.Subscriber(
                "/bebop/image_raw/compressedDepth", CompressedImage,
                self._process_compressed_depth_image),
            'theora': rospy.Subscriber(
                "/bebop/image_raw/theora", CompressedImage,
                self._process_theora_image),
            'camera_orientation': rospy.Subscriber(
                "/bebop/states/ardrone3/CameraState/Orientation",
                Ardrone3CameraStateOrientation,
                self._process_camera_orientation)
        }
        self.param_listener.init_subscribers(['compressed_description',
                                              'compressed_update'])
        return topics

    def _time_to_update(self) -> bool:
        """
        Check if enough time has passed for the next camera operation update.
        """
        if time.time() - self.last_update_time >= self.period:
            self.last_update_time = time.time()
            return True
        return False

    def _process_raw_image(self, data: Image) -> None:
        """Process raw image data from the ROS topic."""
        if self._time_to_update():
            self._save_image_data(data, "image_raw.png", 'image',
                                  use_cv_bridge=True)

    def _process_compressed_image(self, data: CompressedImage) -> None:
        """Process compressed image data from the ROS topic."""
        if self._time_to_update():
            self._save_image_data(data, "compressed.png", 'compressed')

    def _process_compressed_depth_image(self, data: CompressedImage) -> None:
        """Process compressed depth image data from the ROS topic."""
        if self._time_to_update():
            self._save_image_data(data, "depth.png", 'depth')

    def _process_theora_image(self, data: CompressedImage) -> None:
        """Process Theora-encoded image data from the ROS topic."""
        if self._time_to_update():
            self._save_image_data(data, "theora.png", 'theora')

    def _process_camera_orientation(self, data: Ardrone3CameraStateOrientation
                                    ) -> None:
        """Update camera orientation from the ROS topic."""
        if self._time_to_update():
            self.orientation['tilt'] = data.tilt
            self.orientation['pan'] = data.pan

    def _save_image_data(self, data, filename: str, img_type: str,
                         use_cv_bridge: bool = False) -> None:
        """
        Save image data from the ROS topic using the appropriate decoding
        mechanism.

        :param data: Image data from ROS.
        :param filename: Filename to save the image.
        :param img_type: Type of the image (e.g., 'image', 'compressed').
        :param use_cv_bridge: Flag to indicate if CvBridge should be used for
                              conversion.
        """
        try:
            image = (self.bridge.imgmsg_to_cv2(data, "bgr8") if use_cv_bridge
                     else cv2.imdecode(np.frombuffer(data.data, np.uint8
                                                     ), cv2.IMREAD_COLOR))
            self.image_data[img_type] = image
            self.success_flags[img_type] = image is not None
        except (cv2.error, ValueError) as e:
            rospy.logerr(f"Failed to process {img_type} image: {e}")

    def control_camera_orientation(self, tilt: float, pan: float) -> None:
        """
        Set the camera orientation.

        :param tilt: Camera tilt angle.
        :param pan: Camera pan angle.
        """
        control_msg = Twist()
        control_msg.angular.y = tilt
        control_msg.angular.z = pan
        self.pubs['camera_control'].publish(control_msg)

    def capture_snapshot(self) -> None:
        """Capture a snapshot with the drone's camera."""
        self.pubs['snapshot'].publish(Empty())

    def adjust_exposure(self, exposure: float) -> None:
        """Adjust the camera exposure setting."""
        self.pubs['set_exposure'].publish(Float32(data=exposure))


class ParameterListener:
    """Listens to dynamic parameter updates for the Bebop camera."""

    def __init__(self, camera: CameraControl) -> None:
        """
        Initialize the ParameterListener with references to the CameraControl.
        """
        self.camera = camera
        self.subscribers = {}

    def init_subscribers(self, topics: List[str]) -> None:
        """
        Initialize dynamic reconfiguration subscribers for parameter updates.
        """
        param_topics = {
            'compressed_description': (
                "/bebop/image_raw/compressed/parameter_descriptions",
                ConfigDescription, self._param_desc_callback),
            'compressed_update': (
                "/bebop/image_raw/compressed/parameter_updates", Config,
                self._param_update_callback)
        }
        for topic in topics:
            if topic in param_topics:
                topic_name, msg_type, callback = param_topics[topic]
                self.subscribers[topic] = rospy.Subscriber(topic_name,
                                                           msg_type, callback)

    def _param_desc_callback(self, data: ConfigDescription) -> None:
        """Callback to log parameter descriptions."""
        for group in data.groups:
            rospy.loginfo(f"Parameter group: {group.name}")
            for param in group.parameters:
                rospy.logdebug(f" Parameter: {param.name}, Type: {param.type}")

    def _param_update_callback(self, data: Config) -> None:
        """Callback to update camera parameters based on new configuration."""
        self._apply_new_parameters(data.doubles)

    def _apply_new_parameters(self, parameters: list) -> None:
        """Apply updated camera parameters."""
        for param in parameters:
            if param.name == "compression_quality":
                rospy.loginfo(f"Setting compression quality to {param.value}")
                self.camera.compression_quality = param.value
