# File: DroneCamera.py

This module defines the `DroneCamera` class, which is responsible for managing camera operations for the Bebop drone. It handles tasks such as capturing images, adjusting camera orientation, and controlling camera exposure. It follows the Singleton design pattern to ensure a single instance of the camera throughout the application.

---

## Public Methods:

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton design pattern for the `DroneCamera` class, ensuring only one instance is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, main_dir: str, frequency: int = 30)`**
   - Initializes the `DroneCamera` instance with necessary ROS topics, publishers, and subscribers. It also sets the base directory for saving images, and configures camera-related attributes.
   - Parameters:
     - `drone_type`: Type of the drone (e.g., "Bebop2").
     - `main_dir`: The base directory where images will be saved.
     - `frequency`: The frequency (in Hz) for processing camera updates (default: 30 Hz).

3. **`_setup_camera(self)`**
   - Configures the camera by initializing ROS publishers, subscribers, and parameter listeners. If initialization fails, it logs an error and shuts down the ROS node.

4. **`_initialize_publishers(self) -> Dict[str, rospy.Publisher]`**
   - Initializes and returns a dictionary of ROS publishers for camera control, snapshot capture, and exposure adjustment.

5. **`_initialize_subscribers(self) -> Dict[str, rospy.Subscriber]`**
   - Initializes and returns a dictionary of ROS subscribers for topics related to image data and camera orientation, based on the specified drone type.

6. **`_get_msg_type(key: str)`**
   - A static method that returns the appropriate ROS message type for a given topic key (e.g., 'image', 'compressed', 'camera_orientation').

7. **`_get_callback(self, key: str)`**
   - A method that returns the appropriate callback function for a given topic key. The callbacks handle processing of image data or updating camera orientation.

8. **`_time_to_update(self) -> bool`**
   - Determines if it's time to process the next camera update based on the command update frequency and time elapsed since the last update.

9. **`_process_raw_image(self, data: Image) -> None`**
   - Processes raw image data received from the corresponding ROS topic. It converts the image to a usable format using `CvBridge` and stores it.

10. **`_process_compressed_image(self, data: CompressedImage) -> None`**
    - Processes compressed image data received from the corresponding ROS topic. It decodes the compressed image and stores it.

11. **`_update_orientation(self, data: Ardrone3CameraStateOrientation) -> None`**
    - Updates the camera orientation (tilt and pan) based on data received from the ROS topic.

12. **`_process_image(self, data, img_type: str, use_cv_bridge: bool = False) -> None`**
    - A helper method that processes and stores image data. It either uses `CvBridge` to convert raw image data or decodes compressed image data into a usable format.
    - Parameters:
      - `data`: The ROS message containing image data.
      - `img_type`: The type of image (e.g., 'image', 'compressed').
      - `use_cv_bridge`: Whether to use `CvBridge` for decoding.

13. **`control_camera_orientation(self, tilt: float, pan: float) -> None`**
    - Sends a command to adjust the camera's tilt and pan orientation by publishing a `Twist` message to the relevant ROS topic.

14. **`capture_snapshot(self, frame: np.ndarray, filename: str) -> None`**
    - Captures a snapshot from the camera and saves it to a file. It also publishes a snapshot request to the ROS topic.
    - Parameters:
      - `frame`: The image frame to be saved.
      - `filename`: The name of the file where the image will be stored.

15. **`adjust_exposure(self, exposure: float) -> None`**
    - Adjusts the camera's exposure level by publishing the new exposure value to the relevant ROS topic.
    - Parameters:
      - `exposure`: The desired exposure value.

16. **`release(self) -> None`**
    - Releases resources used by the camera, including clearing stored image data, resetting success flags, and resetting camera orientation.

---

This documentation provides an overview of the `DroneCamera` class, which facilitates controlling the Bebop drone's camera for tasks like image capture and orientation adjustment. Each method is responsible for handling a specific aspect of the camera operation, from initialization and data processing to exposure control and resource management.