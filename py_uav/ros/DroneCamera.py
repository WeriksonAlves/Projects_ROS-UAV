"""
DroneCamera: This class handles camera operations for the Bebop drone,
including capturing raw images, managing camera orientation, and controlling
exposure settings.

ROS Topics (10):
    - /bebop/image_raw
    - /bebop/image_raw/compressed
    - /bebop/image_raw/compressed/parameter_descriptions
    - /bebop/image_raw/compressed/parameter_updates
    - /bebop/image_raw/compressedDepth
    - /bebop/image_raw/theora
    - /bebop/camera_control
    - /bebop/states/ardrone3/CameraState/Orientation
    - /bebop/set_exposure
    - /bebop/snapshot

Missing Topics (5):
    - /bebop/camera_info
    - /bebop/image_raw/compressedDepth/parameter_descriptions
    - /bebop/image_raw/compressedDepth/parameter_updates
    - /bebop/image_raw/theora/parameter_descriptions
    - /bebop/image_raw/theora/parameter_updates
"""

import cv2
import numpy as np
import os
import rospy
from ..interfaces.RosCommunication import RosCommunication
from bebop_msgs.msg import Ardrone3CameraStateOrientation
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import ConfigDescription, Config
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty, Float32
from typing import List, Dict


class DroneCamera(RosCommunication):
    """
    Manages camera operations for the Bebop drone, including capturing images,
    managing camera orientation, and controlling exposure settings.
    """

    _instance = None  # Class-level variable to hold the singleton instance

    def __new__(cls, *args, **kwargs):
        """Override __new__ to implement the Singleton pattern."""
        if cls._instance is None:
            cls._instance = super(DroneCamera, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, main_dir: str, frequency: int = 30):
        """
        Initializes the DroneCamera class with publishers, subscribers,
        and image handling configurations.

        :param drone_type: Type of the drone (e.g., "Bebop2").
        :param frequency: Frequency of camera operations (default: 30 Hz).
        """
        if hasattr(self, '_initialized') and self._initialized:
            return  # Prevent reinitialization if instance already exists

        super().__init__(drone_type, frequency)
        self.base_filename = os.path.join(main_dir, 'images', 'internal')
        self.last_command_time = rospy.get_time()

        self.image_data = {key: None for key in ['image', 'compressed',
                                                 'depth', 'theora']}
        self.success_flags = {key: False for key in self.image_data.keys()}
        self.bridge = CvBridge()
        self.orientation = {'tilt': 0.0, 'pan': 0.0}

        try:
            self.param_listener = ParameterListener(self.drone_type, self)
            self.pubs = self._initialize_publishers()
            self.subs = self._initialize_subscribers()
            self.open_camera = True
            rospy.loginfo(f"DroneCamera initialized for {self.drone_type}.")
        except rospy.ROSException as e:
            self.open_camera = False
            rospy.logerr(f"Failed to initialize DroneCamera: {e}")
            quit()

        self._initialized = True  # Mark the instance as initialized

    def _initialize_publishers(self) -> Dict[str, rospy.Publisher]:
        """Initialize ROS publishers for camera commands."""
        return {
            'camera_control': rospy.Publisher('/bebop/camera_control', Twist,
                                              queue_size=10),
            'snapshot': rospy.Publisher('/bebop/snapshot', Empty,
                                        queue_size=10),
            'set_exposure': rospy.Publisher('/bebop/set_exposure', Float32,
                                            queue_size=10)
        }

    def _initialize_subscribers(self) -> Dict[str, rospy.Subscriber]:
        """Initialize ROS subscribers for camera data and orientation."""
        if self.drone_type == "Gazebo":
            return {
                'image': rospy.Subscriber("/bebop2/camera_base/image_raw",
                                          Image, self._process_raw_image),
                'compressed': rospy.Subscriber(
                    "/bebop2/camera_base/image_raw/compressed",
                    CompressedImage, self._process_compressed_image),
                'depth': rospy.Subscriber(
                    "/bebop2/camera_base/image_raw/compressedDepth",
                    CompressedImage, self._process_compressed_depth_image),
                'theora': rospy.Subscriber(
                    "/bebop2/camera_base/image_raw/theora", CompressedImage,
                    self._process_theora_image)
            }
        elif self.drone_type == "Bebop2":
            return {
                'image': rospy.Subscriber("/bebop/image_raw", Image,
                                          self._process_raw_image),
                'compressed': rospy.Subscriber("/bebop/image_raw/compressed",
                                               CompressedImage,
                                               self._process_compressed_image),
                'depth': rospy.Subscriber(
                    "/bebop/image_raw/compressedDepth", CompressedImage,
                    self._process_compressed_depth_image),
                'theora': rospy.Subscriber("/bebop/image_raw/theora",
                                           CompressedImage,
                                           self._process_theora_image),
                'camera_orientation': rospy.Subscriber(
                    "/bebop/states/ardrone3/CameraState/Orientation",
                    Ardrone3CameraStateOrientation, self._update_orientation)
            }
        else:
            raise ValueError("Invalid drone type for camera initialization.")

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
        Saves image data from the ROS topic using the appropriate decoding.

        :param data: Image data from ROS.
        :param img_type: Type of the image (e.g., 'image', 'compressed').
        :param use_cv_bridge: Flag to indicate if CvBridge should be used.
        """
        try:
            image = (self.bridge.imgmsg_to_cv2(data, "bgr8") if use_cv_bridge
                     else cv2.imdecode(np.frombuffer(data.data, np.uint8),
                                       cv2.IMREAD_COLOR))
            self.image_data[img_type] = image
            self.success_flags[img_type] = image is not None

        except (cv2.error, ValueError) as e:
            rospy.logerr(f"Failed to process {img_type} image: {e}")

    def control_camera_orientation(self, tilt: float, pan: float) -> None:
        """Sets the camera orientation."""
        control_msg = Twist()
        control_msg.angular.y = tilt
        control_msg.angular.z = pan
        self.pubs['camera_control'].publish(control_msg)

    def capture_snapshot(self, frame: np.ndarray, filename: str) -> None:
        """
        Captures a snapshot using the drone's camera and saves it to a file.

        :param frame: The image frame to be saved.
        :param filename: The filename to save the snapshot.
        """
        self.pubs['snapshot'].publish(Empty())
        cv2.imwrite(filename, frame)

    def adjust_exposure(self, exposure: float) -> None:
        """Adjusts the camera exposure setting."""
        self.pubs['set_exposure'].publish(Float32(data=exposure))

    def release(self) -> None:
        """Releases the camera resources."""
        self.image_data.clear()
        self.success_flags.clear()
        self.orientation = {'tilt': 0.0, 'pan': 0.0}
        self.control_camera_orientation(0.0, 0.0)
        self.param_listener.subscribers.clear()


class ParameterListener:
    """Listens to dynamic parameter updates for the Bebop camera."""

    def __init__(self, drone_type: str, camera: DroneCamera) -> None:
        """
        Initializes the ParameterListener with references to the DroneCamera.
        """
        self.drone_type = drone_type
        self.camera = camera
        self.subscribers = self._initialize_subscribers()

    def _initialize_subscribers(self) -> None:
        """
        Initialize dynamic reconfiguration subscribers for parameter updates.
        """
        if self.drone_type == "Gazebo":
            return {
                'compressed_description': rospy.Subscriber(
                    "/bebop2/camera_base/image_raw/compressed/parameter_descriptions",
                    ConfigDescription, self._param_desc_callback),
                'compressed_update': rospy.Subscriber(
                    "/bebop2/camera_base/image_raw/compressed/parameter_updates"
                    , Config, self._param_update_callback)
            }
        elif self.drone_type == "Bebop2":
            return {
                'compressed_description': rospy.Subscriber(
                    "/bebop/image_raw/compressed/parameter_descriptions",
                    ConfigDescription, self._param_desc_callback),
                'compressed_update': rospy.Subscriber(
                    "/bebop/image_raw/compressed/parameter_updates", Config,
                    self._param_update_callback)
            }
        else:
            raise ValueError("Invalid drone type for parameter initialization.")

    def _param_desc_callback(self, data: ConfigDescription) -> None:
        """Logs parameter descriptions."""
        for group in data.groups:
            rospy.loginfo(f"Parameter group: {group.name}")
            for param in group.parameters:
                rospy.logdebug(f" Parameter: {param.name}, Type: {param.type}")

    def _param_update_callback(self, data: Config) -> None:
        """Applies camera parameter updates based on new configuration."""
        self._apply_new_parameters(data.doubles)

    def _apply_new_parameters(self, parameters: List[float]) -> None:
        """Applies updated camera parameters."""
        for param in parameters:
            if param.name == "compression_quality":
                rospy.loginfo(f"Setting compression quality to {param.value}")
                self.camera.compression_quality = param.value
