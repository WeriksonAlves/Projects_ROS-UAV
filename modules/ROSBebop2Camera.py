from bebop_msgs.msg import Ardrone3CameraStateOrientation
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import ConfigDescription, Config
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty, Float32
from typing import List, Optional

import cv2
import numpy as np
import os
import rospy
import time


class Bebop2Camera:
    """
    DroneCamera handles camera operations, including capturing images,
    managing camera orientation, and controlling exposure settings via ROS
    topics.
    """

    def __init__(self, file_path: str, fps: int = 30):
        """
        Initialize the DroneCamera object with publishers, subscribers, and
        image handling.
        """
        self.file_path = file_path
        self.fps = fps
        self.current_time = time.time()

        self.image_data = {key: None for key in ['image', 'compressed',
                                                 'depth', 'theora']}
        self.success_flags = {key: False for key in self.image_data.keys()}
        self.success_flags['isOpened'] = False
        self.current_tilt = 0.0
        self.current_pan = 0.0

        self.param_listener = ParameterListener(self)
        self.bridge = CvBridge()

        self.pubs = {}
        self.subs = {}

        rospy.loginfo("DroneCamera initialized.")

    def init_publishers(self, topics: List[str]) -> dict:
        """
        Factory method to initialize ROS publishers.

        :param topics: List of ROS topics to publish to ['camera_control',
            'snapshot', 'set_exposure', 'compressed_description',
            'compressed_update'].
        """
        pub_map = {
            'camera_control': rospy.Publisher('/bebop/camera_control',
                                              Twist, queue_size=10),
            'snapshot': rospy.Publisher('/bebop/snapshot',
                                        Empty, queue_size=10),
            'set_exposure': rospy.Publisher('/bebop/set_exposure',
                                            Float32, queue_size=10)
        }
        return {topic: pub_map[topic] for topic in topics if topic in pub_map}

    def init_subscribers(self, topics: List[str]) -> dict:
        """
        Factory method to initialize ROS subscribers.

        :param topics: List of ROS topics to subscribe to ['image',
            'compressed', 'depth', 'theora'].
        """
        topic_map = {
            'image': ("/bebop/image_raw", Image, self._process_raw_image),
            'compressed': ("/bebop/image_raw/compressed", CompressedImage,
                           self._process_compressed_image),
            'depth': ("/bebop/image_raw/compressedDepth", CompressedImage,
                      self._process_compressed_depth_image),
            'theora': ("/bebop/image_raw/theora", CompressedImage,
                       self._process_theora_image),
        }

        subscribers = {
            topic: rospy.Subscriber(
                topic_map[topic][0], topic_map[topic][1], topic_map[topic][2]
            ) for topic in topics if topic in topic_map
        }

        # Camera orientation subscriber
        subscribers['camera_orientation'] = rospy.Subscriber(
            "/bebop/states/ardrone3/CameraState/Orientation",
            Ardrone3CameraStateOrientation,
            self._process_camera_orientation
        )
        self.param_listener.init_subscribers(topics)
        return subscribers

    def _should_process_frame(self) -> bool:
        """Check if the time interval has passed to process the next frame."""
        if time.time() - self.current_time > (1 / self.fps):
            self.current_time = time.time()
            return True
        return False

    def _process_raw_image(self, data: Image) -> None:
        """Process and save raw image data."""
        if self._should_process_frame():
            self._save_and_load_image(data, "image_raw.png", "image",
                                      use_cv_bridge=True)

    def _process_compressed_image(self, data: CompressedImage) -> None:
        """Process and save compressed image data."""
        if self._should_process_frame():
            self._save_and_load_image(data, "compressed.png", "compressed")

    def _process_compressed_depth_image(self, data: CompressedImage) -> None:
        """Process and save compressed depth image data."""
        if self._should_process_frame():
            self._save_and_load_image(data, "depth.png", "depth")

    def _process_theora_image(self, data: CompressedImage) -> None:
        """Process and save Theora-encoded image data."""
        if self._should_process_frame():
            self._save_and_load_image(data, "theora.png", "theora")

    def _process_camera_orientation(self, data: Ardrone3CameraStateOrientation
                                    ) -> None:
        """Update camera orientation based on ROS topic."""
        if self._should_process_frame():
            self.current_tilt = data.tilt
            self.current_pan = data.pan

    def _save_and_load_image(self, data: CompressedImage, filename: str,
                             img_type: str, use_cv_bridge=False) -> None:
        """
        Save and load image data using the appropriate decoding mechanism.
        """
        try:
            # Decode image from ROS message
            image = self.bridge.imgmsg_to_cv2(
                data, "bgr8") if use_cv_bridge else cv2.imdecode(
                    np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)

            # Save image and update status
            img_path = os.path.join(self.file_path, filename)
            self.success_flags[img_type] = self._save_image(image, img_path)
            self.image_data[img_type] = self._load_image(img_path, img_type)

        except (cv2.error, ValueError) as e:
            rospy.logerr(f"Failed to process {img_type} image: {e}")

    def _save_image(self, image: np.ndarray, filename: str) -> bool:
        """Save an image to disk."""
        return cv2.imwrite(filename, image) if image is not None else False

    def _load_image(self, filename: str, img_type: str
                    ) -> Optional[np.ndarray]:
        """Load an image from a file."""
        return cv2.imread(filename) if self.success_flags[img_type] else None

    def pan_tilt_camera(self, tilt: float, pan: float) -> None:
        """Control the camera orientation."""
        camera_control_msg = Twist()
        camera_control_msg.angular.y = tilt
        camera_control_msg.angular.z = pan
        self.pubs['camera_control'].publish(camera_control_msg)

    def take_snapshot(self) -> None:
        """Command the drone to take a snapshot."""
        self.pubs['snapshot'].publish(Empty())

    def set_exposure(self, exposure_value: float) -> None:
        """Adjust the camera exposure."""
        exposure_msg = Float32(data=exposure_value)
        self.pubs['set_exposure'].publish(exposure_msg)


class ParameterListener:
    """
    Listens to parameter updates for dynamic reconfiguration of camera
    parameters.
    """

    def __init__(self, drone_camera: Bebop2Camera) -> None:
        """Initialize the listener and set up ROS subscribers."""
        self.drone_camera = drone_camera
        self.subs = {}

    def init_subscribers(self, topics: List[str]) -> None:
        """Initialize subscribers for dynamic reconfiguration."""
        topic_map = {
            'compressed_description': (
                "/bebop/image_raw/compressed/parameter_descriptions",
                ConfigDescription, self._callback_param_desc),
            'compressed_update': (
                "/bebop/image_raw/compressed/parameter_updates", Config,
                self._callback_param_update)
        }
        for topic in topics:
            if topic in topic_map:
                topic_name, msg_type, callback = topic_map[topic]
                self.subs[topic] = rospy.Subscriber(topic_name, msg_type,
                                                    callback)

    def _callback_param_desc(self, data: ConfigDescription) -> None:
        """Callback for parameter descriptions."""
        for group in data.groups:
            rospy.loginfo(f"Parameter group: {group.name}")
            for param in group.parameters:
                rospy.logdebug(f" Parameter: {param.name}, Type: {param.type}")

    def _callback_param_update(self, data: Config) -> None:
        """Callback for parameter updates."""
        self._update_parameters(data.doubles)

    def _update_parameters(self, parameters: list) -> None:
        """Update camera settings based on received parameters."""
        for param in parameters:
            if param.name == "compression_quality":
                self._update_compression_quality(param.value)

    def _update_compression_quality(self, value: float) -> None:
        """Update the compression quality of the drone camera."""
        rospy.loginfo(f"Updating compression quality to {value}")
        self.drone_camera.compression_quality = value
