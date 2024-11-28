"""
DroneCamera: Handles camera operations for the Bebop drone, including image
                capture, camera orientation, and exposure control.
"""

from ..interfaces.RosCommunication import RosCommunication
from bebop_msgs.msg import Ardrone3CameraStateOrientation
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty, Float32
from typing import Dict
import cv2
import numpy as np
import os
import rospy


class DroneCamera(RosCommunication):
    """
    Singleton class managing camera operations for the Bebop drone.
    """

    _instance = None  # Singleton instance

    def __new__(cls, *args, **kwargs):
        """Implements the Singleton design pattern."""
        if cls._instance is None:
            cls._instance = super(DroneCamera, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, main_dir: str, frequency: int = 30):
        """
        Initialize the DroneCamera instance with ROS topics, publishers, and
        subscribers.

        :param drone_type: Type of drone (e.g., "Bebop2").
        :param main_dir: Base directory for saving images.
        :param frequency: Frequency for processing camera updates (default: 30
                            Hz).
        """
        if getattr(self, '_initialized', False):
            return  # Avoid reinitialization

        super().__init__(drone_type, frequency)
        self.base_filename = os.path.join(main_dir, 'images', 'internal')
        self.last_command_time = rospy.get_time()
        self.image_data = {key: None for key in ['image', 'compressed',
                                                 'depth', 'theora']}
        self.success_flags = {key: False for key in self.image_data}
        self.bridge = CvBridge()
        self.orientation = {'tilt': 0.0, 'pan': 0.0}
        self.open_camera = False

        self._setup_camera()

        self._initialized = True  # Mark instance as initialized

    def _setup_camera(self):
        """Set up camera publishers, subscribers, and parameter listeners."""
        try:
            self.pubs = self._initialize_publishers()
            self.subs = self._initialize_subscribers()
            self.open_camera = True
            rospy.loginfo(f"DroneCamera initialized for {self.drone_type}.")
        except rospy.ROSException as err:
            rospy.logerr(f"Failed to initialize DroneCamera: {err}")
            rospy.signal_shutdown("Critical error in DroneCamera setup.")

    def _initialize_publishers(self) -> Dict[str, rospy.Publisher]:
        """Create ROS publishers for camera operations."""
        return {
            'camera_control': rospy.Publisher('/bebop/camera_control', Twist,
                                              queue_size=10),
            'snapshot': rospy.Publisher('/bebop/snapshot', Empty,
                                        queue_size=10),
            'set_exposure': rospy.Publisher('/bebop/set_exposure', Float32,
                                            queue_size=10)
        }

    def _initialize_subscribers(self) -> Dict[str, rospy.Subscriber]:
        """Subscribe to camera-related topics based on the drone type."""
        topic_mapping = {
            'gazebo': {
                'image': "/bebop2/camera_base/image_raw",
                'compressed': "/bebop2/camera_base/image_raw/compressed",
                'depth': "/bebop2/camera_base/image_raw/compressedDepth",
                'theora': "/bebop2/camera_base/image_raw/theora",
            },
            'bebop2': {
                'image': "/bebop/image_raw",
                'compressed': "/bebop/image_raw/compressed",
                'depth': "/bebop/image_raw/compressedDepth",
                'theora': "/bebop/image_raw/theora",
                'camera_orientation':
                    "/bebop/states/ardrone3/CameraState/Orientation",
            },
        }

        topics = topic_mapping.get(self.drone_type.lower())
        if not topics:
            raise ValueError(f"Unsupported drone type: {self.drone_type}")

        return {
            key: rospy.Subscriber(topic, self._get_msg_type(key),
                                  self._get_callback(key))
            for key, topic in topics.items()
        }

    @staticmethod
    def _get_msg_type(key: str):
        """Return the message type for a given topic key."""
        msg_types = {
            'image': Image,
            'compressed': CompressedImage,
            'depth': CompressedImage,
            'theora': CompressedImage,
            'camera_orientation': Ardrone3CameraStateOrientation,
        }
        return msg_types[key]

    def _get_callback(self, key: str):
        """Return the appropriate callback function for the given topic key."""
        callbacks = {
            'image': self._process_raw_image,
            'compressed': self._process_compressed_image,
            'depth': self._process_compressed_depth_image,
            'theora': self._process_theora_image,
            'camera_orientation': self._update_orientation,
        }
        return callbacks[key]

    def _time_to_update(self) -> bool:
        """Determine if it's time to process the next camera update."""
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    def _process_raw_image(self, data: Image) -> None:
        """Processes raw image data from the ROS topic."""
        if self._time_to_update():
            self._process_image(data, 'image',
                                use_cv_bridge=True)

    def _process_compressed_image(self, data: CompressedImage) -> None:
        """Processes compressed image data from the ROS topic."""
        if self._time_to_update():
            self._process_image(data, 'compressed')

    def _process_compressed_depth_image(self, data: CompressedImage) -> None:
        """Processes compressed depth image data from the ROS topic."""
        if self._time_to_update():
            self._process_image(data, 'depth')

    def _process_theora_image(self, data: CompressedImage) -> None:
        """Processes Theora-encoded image data from the ROS topic."""
        if self._time_to_update():
            self._process_image(data, 'theora')

    def _update_orientation(self, data: Ardrone3CameraStateOrientation
                            ) -> None:
        """Updates the camera orientation from the ROS topic."""
        if self._time_to_update():
            self.orientation.update({'tilt': data.tilt, 'pan': data.pan})

    def _process_image(self, data, img_type: str, use_cv_bridge: bool = False
                       ) -> None:
        """
        Process and store image data from the camera topics.

        :param data: ROS message data.
        :param img_type: Image type (e.g., 'image', 'compressed').
        :param use_cv_bridge: Whether to use CvBridge for image decoding.
        """
        try:
            if use_cv_bridge:
                image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            else:
                image = cv2.imdecode(np.frombuffer(data.data, np.uint8),
                                     cv2.IMREAD_COLOR)
            self.image_data[img_type] = image
            self.success_flags[img_type] = image is not None
        except (cv2.error, ValueError) as err:
            rospy.logerr(f"Error processing {img_type} image: {err}")

    def control_camera_orientation(self, tilt: float, pan: float) -> None:
        """Sets the camera orientation."""
        control_msg = Twist()
        control_msg.angular.y = tilt
        control_msg.angular.z = pan
        self.pubs['camera_control'].publish(control_msg)

    def capture_snapshot(self, frame: np.ndarray, filename: str) -> None:
        """Save a snapshot from the camera."""
        self.pubs['snapshot'].publish(Empty())
        cv2.imwrite(filename, frame)

    def adjust_exposure(self, exposure: float) -> None:
        """Set the camera's exposure level."""
        self.pubs['set_exposure'].publish(Float32(data=exposure))

    def release(self) -> None:
        """Release resources used by the camera."""
        self.image_data.clear()
        self.success_flags.clear()
        self.orientation = {'tilt': 0.0, 'pan': 0.0}
        self.control_camera_orientation(0.0, 0.0)
