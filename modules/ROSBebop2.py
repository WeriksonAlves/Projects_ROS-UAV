from .ROSBebop2Camera import Bebop2Camera
from .ROSBebop2Control import Bebop2Control
from .ROSBebop2Sensors import Bebop2Sensors
from typing import Callable, List, Optional, Tuple

import os
import rospy
import numpy as np


class Bebop2:
    """
    Bebop2 manages communication with the Bebop drone, handling video capture,
    sensor updates, and camera control via ROS.
    """

    def __init__(self, drone_type: str = "Bebop2",
                 ip_address: str = "192.168.0.202"):
        """
        Initialize the Bebop2 object with drone type, IP address, camera,
        control system, and sensor handling.

        :param drone_type: Type of drone, default is 'Bebop2'.
        :param ip_address: The drone's IP address, default is '192.168.0.202'.
        """
        self.drone_type = drone_type
        self.ip_address = ip_address
        self.main_dir = os.path.dirname(__file__)

        # Drone state mappings
        self.state_map = {
            'E': 'emergency',
            'H': 'hovering',
            'L': 'landed',
            'l': 'landing',
            'M': 'moving',
            'T': 'takingoff'
        }

        # Initialize drone components
        self.camera = Bebop2Camera(os.path.join(self.main_dir, 'images'),
                                   fps=10)
        self.control = Bebop2Control(drone_type)
        self.sensors = Bebop2Sensors()

    # Sensor Management Methods

    def set_sensor_callback(self, callback_fn: Callable, args: tuple) -> None:
        """
        Set a callback function for sensor updates.

        :param callback_fn: Function to be called when sensors are updated.
        :param args: Arguments for the callback function.
        """
        self.sensors.set_user_callback_function(callback_fn, args)

    def update_sensors(self, *args) -> None:
        """
        Placeholder method to handle sensor updates.
        To be implemented if needed.
        """
        pass

    # Camera management methods

    def start_video_capture(self, publishers: List[str] = None,
                            subscribers: List[str] = None) -> None:
        """
        Initialize video capture, set up publishers and subscribers for ROS
        topics.

        :param publishers: ROS topics to publish to. Default is camera control.
        :param subscribers: ROS topics to subscribe to. Default is compressed
        image data.
        """
        publishers = publishers or ['camera_control']
        subscribers = subscribers or ['compressed', 'compressed_description',
                                      'compressed_update']

        self._ensure_directory_exists()
        try:
            self.camera.init_publishers(publishers)
            rospy.loginfo(f"Initialized publishers: {publishers}")

            self.camera.init_subscribers(subscribers)
            rospy.loginfo(f"Initialized subscribers: {subscribers}")

            self.camera.success_flags["isOpened"] = True
            rospy.loginfo("Camera successfully initialized and opened.")
        except Exception as e:
            self.camera.success_flags["isOpened"] = False
            rospy.logerr(f"Error during video capture initialization: {e}")

    def _ensure_directory_exists(self) -> None:
        """Ensure that the image storage directory exists."""
        if not os.path.exists(self.camera.file_path):
            os.makedirs(self.camera.file_path)
            rospy.loginfo(f"Created directory: {self.camera.file_path}")

    def is_camera_opened(self) -> bool:
        """
        Check if the camera is opened.

        :return: True if the camera is active, False otherwise.
        """
        return self.camera.success_flags["isOpened"]

    def capture_frame(self, subscriber: str = 'compressed'
                      ) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Capture the latest image frame from the camera.

        :param subscriber: The subscriber to read from (default: 'compressed').
        :return: Tuple containing success flag and the image data.
        """
        return self.camera.success_flags[
            subscriber], self.camera.image_data.get(subscriber)

    def release_camera(self) -> None:
        """Release camera resources and reset internal state."""
        self.camera.success_flags = {
            key: False for key in self.camera.success_flags}
        self.camera.image_data = {key: None for key in self.camera.image_data}
        rospy.loginfo("Camera resources released.")

    def adjust_camera(self, tilt: float, pan: float, pitch: float = 0.0,
                      yaw: float = 0.0) -> None:
        """
        Adjust the camera's tilt and pan, compensating for pitch and yaw.

        :param tilt: Vertical camera movement.
        :param pan: Horizontal camera movement.
        :param pitch: Compensation for drone pitch.
        :param yaw: Compensation for drone yaw.
        """
        self.camera.pan_tilt_camera(tilt=tilt - pitch, pan=pan - yaw)

    def take_snapshot(self, frame: np.ndarray) -> None:
        """
        Save the current frame as a snapshot.

        :param frame: Image frame to save.
        """
        snapshot_path = os.path.join(self.camera.file_path, 'snapshot.png')
        self.camera._save_image(frame, snapshot_path)

    def adjust_exposure(self, exposure_value: float) -> None:
        """
        Adjust the camera's exposure setting.

        :param exposure_value: Exposure value to set (-3.0 to 3.0).
        """
        self.camera.set_exposure(exposure_value)

    # Drone control methods

    def _is_emergency(self) -> bool:
        """Check if the drone is in an emergency state."""
        return self.sensors.flying_state == self.state_list['E']

    def _is_landed(self) -> bool:
        """Check if the drone is in a landed state."""
        return self.sensors.flying_state == self.state_list['L']

    def _is_hovering(self) -> bool:
        """Check if the drone is hovering."""
        return self.sensors.flying_state == self.state_list['H']

    def takeoff(self) -> None:
        """
        Sends a takeoff command to the drone. If the drone is in an emergency
        state or already flying, it will not proceed.
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_landed():
            self.control.takeoff()
            self.sensors.flying_state = self.state_list['H']
            rospy.loginfo("Bebop is taking off.")
        else:
            rospy.loginfo(
                "Bebop is already flying or performing other actions.")

    def land(self) -> None:
        """
        Sends a land command to the drone. If the drone is in an emergency
        state or already landed, it will not proceed.
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_hovering():
            self.control.land()
            self.sensors.flying_state = self.state_list['L']
            rospy.loginfo("Bebop is landing.")
        else:
            rospy.loginfo(
                "Bebop is already landed or executing other actions.")

    def safe_takeoff(self, timeout: Optional[int] = 5) -> None:
        """
        A safe method for taking off. It ensures the drone takes off and
        reaches the hovering state within the given timeout.

        :param timeout: Maximum time to wait for the drone to take off and
            hover (in seconds).
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_landed():
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < timeout:
                self.control.takeoff()
                rospy.loginfo("Bebop is taking off.")
                self.sensors.flying_state = self.state_list['T']

                rospy.sleep(0.1)

                if self.sensors.sensors_dict['height'] > 0.5:
                    self.sensors.flying_state = self.state_list['H']
                    rospy.loginfo("Bebop is hovering.")
                    break
        else:
            rospy.loginfo(
                "Bebop is already flying or performing other actions.")

    def safe_land(self, timeout: Optional[int] = 5) -> None:
        """
        A safe method for landing. It ensures the drone lands successfully
        within the given timeout.

        :param timeout: Maximum time to wait for the drone to land (seconds).
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_hovering():
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < timeout:
                self.control.land()
                rospy.loginfo("Bebop is landing.")
                self.sensors.flying_state = self.state_list['l']

                rospy.sleep(0.1)

                if self.sensors.sensors_dict['height'] < 0.2:
                    self.sensors.flying_state = self.state_list['L']
                    rospy.loginfo("Bebop is landed.")
                    break
        else:
            rospy.loginfo(
                "Bebop is already landed or performing other actions.")

    def flip(self, direction: str) -> None:
        """
        Sends a flip command to the drone. Valid directions are 'forward',
        'backward', 'left', and 'right'. The drone must be in a hovering state
        to execute the flip.

        :param direction: Direction of the flip ['forward', 'backward', 'left',
            'right'].
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        valid_directions = ['forward', 'backward', 'left', 'right']
        if direction.lower() not in valid_directions:
            rospy.logwarn(f"Invalid flip direction: {direction}. Valid options"
                          f" are {valid_directions}.")
            return

        if not self._is_hovering():
            rospy.logwarn("Bebop must be hovering to flip.")
            return

        self.control.flip(direction)
        rospy.loginfo(f"Flip command executed in {direction} direction.")

    def _normalize_percentage(self, value: float) -> float:
        """Convert a percentage value to a range of -1.0 to 1.0."""
        return value / 100.0

    def fly_direct(self, roll: float, pitch: float, yaw: float,
                   vertical_movement: float, duration: Optional[float] = 0.0) -> None:
        """
        Fly the drone directly using the specified roll, pitch, yaw, and
        vertical movements. The commands are executed for a specified duration.
        If duration is zero, the command is sent only once.

        :param roll: Roll percentage (-100 to 100).
        :param pitch: Pitch percentage (-100 to 100).
        :param yaw: Yaw percentage (-100 to 100).
        :param vertical_movement: Vertical movement percentage (-100 to 100).
        :param duration: Duration in seconds (default: 0.0, no repeat).
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        # Normalize inputs
        roll = self._normalize_percentage(roll)
        pitch = self._normalize_percentage(pitch)
        yaw = self._normalize_percentage(yaw)
        vertical_movement = self._normalize_percentage(vertical_movement)

        if duration == 0:
            self.control.move(roll, pitch, vertical_movement, yaw)
        else:
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < duration:
                self.control.move(roll, pitch, vertical_movement, yaw)
                rospy.sleep(0.1)
            self.control.move(0.0, 0.0, 0.0, 0.0)

    def move_relative(self, dx: float, dy: float, dz: float, dr: float,
                      fator: float = 0.1) -> None:
        """
        Moves the drone relative to its current position. This method assumes
        full or no GPS coverage. Note: Reference is the drone (positive is
        forward, right, down, and clockwise).

        :param dx: Distance to move along the X-axis (forward/backward in
            meters).
        :param dy: Distance to move along the Y-axis (right/left in meters).
        :param dz: Distance to move along the Z-axis (down/up in meters).
        :param dr: Rotation around the Z-axis (in radians).
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        des_pos = [dx, dy, dz, dr]
        cur_pos = self.sensors.sensors_dict['X'][[0, 1, 2, 5]]
        error = des_pos - cur_pos

        # As long as the error in any dimension is greater than 0.1
        while (np.abs(error) >= 0.1).any():
            if np.abs(error[0]) > 0.1:
                self.control.move(1*fator, 0, 0, 0)
            elif np.abs(error[1]) > 0.1:
                self.control.move(0, 1*fator, 0, 0)
            elif np.abs(error[2]) > 0.1:
                self.control.move(0, 0, 1*fator, 0)
            elif np.abs(error[3]) > 0.1:
                self.control.move(0, 0, 0, 1*fator)

        self.control.move(0.0, 0.0, 0.0, 0.0)
        rospy.loginfo(f"Moved drone by dx={dx}, dy={dy}, dz={dz}, dr={dr}.")

    def set_max_altitude(self, altitude: int):
        """
        Set the maximum allowable altitude in meters. The altitude must be
        between 0.5 and 150 meters.
        """
        pass

    def set_max_distance(self, distance: int):
        """
        Set max distance between the takeoff and the drone in meters. The
        distance must be between 10 and 2000 meters.
        """
        pass

    def enable_geofence(self, value: int):
        """
        If geofence is enabled, the drone won't fly over the given max
        distance. Valid value: 1 if the drone can't fly further than max
        distance, 0 if no limitation on the drone should be done.
        """
        pass

    def set_max_tilt(self, tilt: int):
        """
        Set the maximum allowable tilt in degrees for the drone (this limits
        speed). The tilt must be between 5 (very slow) and 30 (very fast)
        degrees.
        """
        pass

    def set_max_tilt_rotation_speed(self, speed: int):
        """
        Set the maximum allowable tilt rotation speed in degree/s. The tilt
        rotation speed must be between 80 and 300 degree/s.
        """
        pass

    def set_max_vertical_speed(self, speed: float):
        """
        Set the maximum allowable vertical speed in m/s. The vertical speed
        must be between 0.5 and 2.5 m/s.
        """
        pass

    def set_max_rotation_speed(self, speed: int):
        """
        Set the maximum allowable rotation speed in degree/s. The rotation
        speed must be between 10 and 200 degree/s.
        """
        pass

    def set_flat_trim(duration: float = 0):
        """
        Tell the Bebop to run with a flat trim. If duration > 0, waits for the
        comand to be acknowledged.
        """
        pass

    """Pausing or sleeping in a thread safe manner"""

    def smart_sleep(self, seconds: float):
        """
        This sleeps the number of seconds (which can be a floating point) but
        wakes for all wifi notifications. You should use this instead of
        time.sleep to be consistent with the mambo but it is not required
        (whereas time.sleep() will break a mambo using BLE).
        """
        pass

    """Video camera"""

    """Sensor commands"""

    def ask_for_state_update(self):
        """
        This sends a request to the bebop to send back ALL states. The data
        returns fairly quickly although not instantly. The bebop already has a
        sensor refresh rate of 10Hz but not all sensors are sent automatically.
        If you are looking for a specific sensor that is not automatically
        sent, you can call this but I don't recommend sending it over and over.
        Most of the sensors you need should be sent at either the 10Hz rate or
        as an event is called that triggers that sensor.
        """
        pass

    """Bebop sensors"""

    #bebop.set_user_sensor_callback(function, args)


class MyFunctions:
    def ajust_camera(self, frame: np.ndarray, bounding_box: np.ndarray,
                     Gi: Tuple[int, int] = (0.5, 0.5), Ge: Tuple[int, int] = (
                         50, 50)) -> None:
        """
        Adjust the camera orientation based on the operator's position in the
        frame.

        :param frame: The captured frame.
        :param boxes: Bounding boxes for detected objects.
        :param Gi: Internal gain for pitch and yaw adjustment.
        :param Ge: External gain for pitch and yaw adjustment.
        """
        dist_center_h, dist_center_v = self.centralize_operator(
            frame, bounding_box)
        sc_pitch = np.tanh(-dist_center_v * Gi[0]) * Ge[0] if abs(
            dist_center_v) > 0.25 else 0
        sc_yaw = np.tanh(dist_center_h * Gi[1]) * Ge[1] if abs(
            dist_center_h) > 0.25 else 0
        print(f"pitch: {sc_pitch}, yaw: {sc_yaw}")
        self.camera.move_camera(tilt=sc_pitch, pan=sc_yaw)

    def centralize_operator(self, frame: np.ndarray, bounding_box: Tuple[
        int, int, int, int], drone_pitch: float = 0.0, drone_yaw: float = 0.0
    ) -> Tuple[float, float]:
        """
        Adjust the camera's orientation to center the operator in the frame,
        compensating for yaw and pitch.

        :param frame: The captured frame.
        :param bounding_box: The bounding box of the operator as (x, y, width,
        height).
        :param drone_pitch: The pitch value for compensation.
        :param drone_yaw: The yaw value for compensation.
        :return: Tuple[float, float]: The horizontal and vertical distance to
        the frame's center.
        """
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)

        box_x, box_y, _, _ = bounding_box
        dist_to_center_h = (
            box_x - frame_center[0]) / frame_center[0] - drone_yaw
        dist_to_center_v = (
            box_y - frame_center[1]) / frame_center[1] - drone_pitch

        return dist_to_center_h, dist_to_center_v
