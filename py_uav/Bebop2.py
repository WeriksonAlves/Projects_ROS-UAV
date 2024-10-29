from .commandsandsensors.SensorsParser import SensorsParser
from .ros.DroneCamera import DroneCamera
from .ros.DroneControl import DroneControl
from .ros.DroneManagers import GPSStateManager, HealthMonitor, ParameterManager
from .ros.DroneMedia import DroneMedia
from .ros.DroneSensors import DroneSensors
from .ros.DroneStates import FlightStateManager
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
        self.camera = DroneCamera(drone_type)
        self.control = DroneControl(drone_type)
        self.gps = GPSStateManager(drone_type)
        self.health = HealthMonitor(drone_type)
        self.params = ParameterManager(drone_type)
        self.media = DroneMedia(drone_type)
        self.sensors = SensorsParser(drone_type)

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

    def smart_sleep(self, seconds: float):
        """
        This sleeps the number of seconds (which can be a floating point) but
        wakes for all wifi notifications. You should use this instead of
        time.sleep to be consistent with the mambo but it is not required
        (whereas time.sleep() will break a mambo using BLE).
        """
        pass

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

    # Drone Control Methods

    def _is_emergency(self) -> bool:
        """Check if the drone is in an emergency state."""
        return self.sensors.flying_state == self.state_map['E']

    def _is_landed(self) -> bool:
        """Check if the drone is in a landed state."""
        return self.sensors.flying_state == self.state_map['L']

    def _is_hovering(self) -> bool:
        """Check if the drone is hovering."""
        return self.sensors.flying_state == self.state_map['H']

    def takeoff(self) -> None:
        """
        Send a takeoff command to the drone if it's in a landed state and not
        in emergency.
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_landed():
            self.control.takeoff()
            self.sensors.flying_state = self.state_map['H']
            rospy.loginfo("Bebop is taking off.")
        else:
            rospy.loginfo("Bebop is already flying or busy.")

    def land(self) -> None:
        """
        Send a land command to the drone if it's hovering and not in emergency.
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_hovering():
            self.control.land()
            self.sensors.flying_state = self.state_map['L']
            rospy.loginfo("Bebop is landing.")
        else:
            rospy.loginfo("Bebop is already landed or busy.")

    def safe_takeoff(self, timeout: int = 5) -> None:
        """
        Safely take off, ensuring the drone reaches the hovering state within
        the timeout.

        :param timeout: Maximum time to wait for the drone to take off
            (seconds).
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_landed():
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < timeout:
                self.control.takeoff()
                rospy.loginfo("Bebop is taking off.")
                self.sensors.flying_state = self.state_map['T']

                rospy.sleep(0.1)

                if self.sensors.get_altitude() > 0.5:
                    self.sensors.flying_state = self.state_map['H']
                    rospy.loginfo("Bebop is hovering.")
                    break
        else:
            rospy.loginfo("Bebop is already flying or busy.")

    def safe_land(self, timeout: int = 5) -> None:
        """
        Safely land, ensuring the drone lands within the timeout.

        :param timeout: Maximum time to wait for the drone to land (seconds).
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
        elif self._is_hovering():
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < timeout:
                self.control.land()
                rospy.loginfo("Bebop is landing.")
                self.sensors.flying_state = self.state_map['l']

                rospy.sleep(0.1)

                if self.sensors.get_altitude() < 0.15:
                    self.sensors.flying_state = self.state_map['L']
                    rospy.loginfo("Bebop has landed.")
                    break
        else:
            rospy.loginfo("Bebop is already landed or busy.")

    def flip(self, direction: str) -> None:
        """
        Perform a flip in the specified direction. Valid directions are
        'forward', 'backward', 'left', and 'right'.

        :param direction: Direction for the flip.
        """
        if self._is_emergency():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if not self._is_hovering():
            rospy.logwarn("Bebop must be hovering to flip.")
            return

        valid_directions = ['forward', 'backward', 'left', 'right']
        if direction.lower() not in valid_directions:
            rospy.logwarn(f"Invalid flip direction: {direction}. Valid options"
                          f" are {valid_directions}.")
            return

        self.control.flip(direction)
        rospy.loginfo(f"Flip command executed in {direction} direction.")

    def _normalize_percentage(self, value: float) -> float:
        """
        Normalize percentage input to a range between -1.0 and 1.0.

        :param value: The input percentage (-100 to 100).
        :return: Normalized value (-1.0 to 1.0).
        """
        if np.abs(value) > 100:
            return 1.0 if value > 0 else -1.0
        return value / 100.0

    def fly_direct(self, roll: float, pitch: float, yaw: float,
                   vertical_movement: float, duration: Optional[float] = 0.0
                   ) -> None:
        """
        Control the drone's movement using roll, pitch, yaw, and vertical
        movements for a specified duration.

        :param roll: Roll percentage (-100 to 100).
        :param pitch: Pitch percentage (-100 to 100).
        :param yaw: Yaw percentage (-100 to 100).
        :param vertical_movement: Vertical movement percentage (-100 to 100).
        :param duration: Duration in seconds for how long the drone should fly
        (default is 0.0, meaning no repeat).
        """
        if self._is_emergency():
            rospy.loginfo("Drone is in emergency mode.")
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
            self.control.move(0.0, 0.0, 0.0, 0.0)  # Stop after the duration

    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                      dr: float = 0.0, velocity: float = 0.1) -> None:
        """
        Move the drone relative to its current position based on distances
        provided along the X, Y, Z axes, and a rotation angle.

        :param dx: Distance to move along the X-axis (in meters, positive is
            forward).
        :param dy: Distance to move along the Y-axis (in meters, positive is
            right).
        :param dz: Distance to move along the Z-axis (in meters, positive is
            down).
        :param dr: Rotation in radians around the Z-axis (positive is
            clockwise).
        :param velocity: The movement velocity (default: 0.1).
        """
        if self._is_emergency():
            rospy.loginfo("Drone is in emergency mode.")
            return

        # Desired position and current position
        desired_position = np.array([dx, dy, dz, dr])
        current_position = np.array(self.sensors.sensors_dict['X'])[[0, 1, 2, 5]]  # Assuming [X, Y, Z, Rotation]
        error = desired_position - current_position

        # Loop until error in any dimension is reduced below 0.1
        while (np.abs(error) >= 0.1).any():
            movement_vector = np.zeros(4)
            if np.abs(error[0]) > 0.1:
                movement_vector[0] = np.sign(error[0]) * velocity
            if np.abs(error[1]) > 0.1:
                movement_vector[1] = np.sign(error[1]) * velocity
            if np.abs(error[2]) > 0.1:
                movement_vector[2] = np.sign(error[2]) * velocity
            if np.abs(error[3]) > 0.1:
                movement_vector[3] = np.sign(error[3]) * velocity

            self.control.move(*movement_vector)
            error = desired_position - np.array(self.sensors.sensors_dict['X'])[[0, 1, 2, 5]]  # Recalculate error

        # Stop movement after reaching the target
        self.control.move(0.0, 0.0, 0.0, 0.0)
        rospy.loginfo(f"Moved drone by dx={dx}, dy={dy}, dz={dz}, dr={dr}.")
