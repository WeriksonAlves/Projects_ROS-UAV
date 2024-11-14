#!/usr/bin/env python3
import os
import numpy as np
import rospy
import threading
from typing import Callable, Tuple

from .commandsandsensors.DroneCommandManager import DroneCommandManager
from .commandsandsensors.DroneSensorManager import DroneSensorManager
from .commandsandsensors.DroneSettingManager import DroneSettingManager


class Bebop2:
    """
    Manages interaction between a Bebop2 drone and ROS, providing control and
    sensor access.
    """

    def __init__(self, drone_type: str = 'bebop2',
                 ip_address: str = "192.168.0.202", frequency: float = 30.0
                 ) -> None:
        """
        Initialize Bebop2ROS with drone type, IP address, and sensor update
        frequency.

        :param drone_type: Type of the drone.
        :param ip_address: IP address of the drone.
        :param frequency: Frequency of sensor updates.
        """
        # Initialize ROS node
        rospy.init_node('BEBOP2', anonymous=True)

        # Initialize drone parameters
        self.drone_type = drone_type
        self.ip_address = ip_address
        self.frequency = frequency

        # Initialize subsystem managers
        self.main_dir = os.path.dirname(os.path.abspath(__file__))
        self.sensor_manager = DroneSensorManager(
            drone_type, frequency, ip_address, self.main_dir)
        self.command_manager = DroneCommandManager(
            drone_type, frequency, self.sensor_manager)
        self.state_manager = DroneSettingManager(
            self.command_manager, self.sensor_manager)

        # User-defined callback
        self.user_callback: Callable = None
        self.user_callback_args = None

        # Initialize thread for sensor updates
        self.sensor_thread = threading.Thread(target=self._sensor_update_loop)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()

    # User-defined callback methods
    def set_user_sensor_callback(self, callback: Callable, *args) -> None:
        """
        Registers a user-defined callback function to handle custom events.

        :param callback: Callable function to register as a callback.
        :param args: Additional arguments for the callback.
        """
        self.user_callback = callback
        self.user_callback_args = args

    def trigger_callback(self) -> None:
        """Executes the user-defined callback with provided arguments."""
        if self.user_callback:
            self.user_callback(*self.user_callback_args)

    # Drone State Methods
    def smart_sleep(self, seconds: float) -> None:
        """Sleep while allowing ROS to continue processing."""
        rospy.sleep(seconds)

    def update_sensors(self) -> None:
        """
        Updates sensor data by invoking the sensor manager's update method.
        """
        self.sensor_manager.update_sensor_data()

    def check_connection(self) -> bool:
        """Checks if the drone is connected to the network."""
        return self.sensor_manager.check_connection()

    # Drone Control Methods
    def takeoff(self) -> None:
        """Commands the drone to take off."""
        self._execute_command(self.command_manager.takeoff, "taking off")

    def safe_takeoff(self, timeout: float = 3.0) -> None:
        """
        Safely takes off the drone by checking altitude.

        :param timeout: Maximum time to attempt takeoff.
        """
        self._execute_command(
            lambda: self.command_manager.safe_takeoff(timeout), "taking off")

    def land(self) -> None:
        """Commands the drone to land."""
        self._execute_command(self.command_manager.land, "landing")

    def emergency_land(self) -> None:
        """Executes an emergency stop."""
        self._execute_command(
            self.command_manager.emergency_stop, "emergency landing")

    def is_landed(self) -> bool:
        """Checks if the drone is landed."""
        return self.sensor_manager.is_landed()

    def safe_land(self, timeout: float = 3.0) -> None:
        """
        Safely lands the drone by checking altitude.

        :param timeout: Maximum time to attempt landing.
        """
        self._execute_command(
            lambda: self.command_manager.safe_land(timeout), "landing")

    def fly_direct(self, linear_x: float = 0.0, linear_y: float = 0.0,
                   linear_z: float = 0.0, angular_z: float = 0.0,
                   duration: float = 0.0) -> None:
        """
        Commands the drone to move directly in the specified direction.

        :param linear_x: Linear velocity in the x-axis.
        :param linear_y: Linear velocity in the y-axis.
        :param linear_z: Linear velocity in the z-axis.
        :param angular_z: Angular velocity in the z-axis.
        :param duration: Duration of movement.
        """
        self._execute_command(
            lambda: self.command_manager.fly_direct(
                self._normalize_command(linear_x),
                self._normalize_command(linear_y),
                self._normalize_command(linear_z),
                self._normalize_command(angular_z),
                duration
            ),
            "moving the drone"
        )

    def flip(self, direction: str) -> None:
        """
        Flips the drone in the specified direction.

        :param direction: Direction of flip ('left', 'right', 'forward',
                          'backward').
        """
        self._execute_command(
            lambda: self.command_manager.flip(direction),
            f"flipping {direction}")

    def move_relative(self, delta_x: float = 0.0, delta_y: float = 0.0,
                      delta_z: float = 0.0, delta_yaw: float = 0.0,
                      power: int = 20) -> None:
        """
        Moves the drone in the specified relative direction.

        :param delta_x: Change in x-axis.
        :param delta_y: Change in y-axis.
        :param delta_z: Change in z-axis.
        :param delta_yaw: Change in yaw.
        :param power: Power of the movement [0 to 100].
        """
        self._execute_command(
            lambda: self.command_manager.move_relative(delta_x, delta_y,
                                                       delta_z, delta_yaw,
                                                       power),
            "moving to relative position"
        )

    # Camera Methods
    def release_camera(self) -> None:
        """Releases the camera resources."""
        self._execute_command(
            self.command_manager.release_camera, "releasing camera")

    def take_snapshot(self) -> None:
        """Captures a snapshot using the drone's camera."""
        self._execute_command(
            self.sensor_manager.take_snapshot, "capturing snapshot")

    def pan_tilt_camera(self, tilt: float, pan: float, pitch_comp: float = 0.0,
                        yaw_comp: float = 0.0) -> None:
        """
        Adjusts the camera orientation.

        :param tilt: Vertical movement in degrees.
        :param pan: Horizontal movement in degrees.
        :param pitch_comp: Optional pitch compensation.
        :param yaw_comp: Optional yaw compensation.
        """
        self._execute_command(
            lambda: self.command_manager.adjust_camera_orientation(
                tilt - pitch_comp, pan - yaw_comp),
            "adjusting camera orientation"
        )

    def adjust_camera_exposure(self, exposure: float) -> None:
        """
        Adjusts the camera exposure setting.

        :param exposure: Exposure level, typically between -3 to 3.
        """
        self._execute_command(
            lambda: self.command_manager.adjust_camera_exposure(exposure),
            "adjusting camera exposure")

    def read_image(self, subscriber: str = 'compressed') -> Tuple[bool,
                                                                  np.ndarray]:
        """
        Reads an image from the drone's camera.

        :param subscriber: The name of the image data subscriber to use.
        :return: A tuple containing a success flag and the image data.
        """
        try:
            return self.sensor_manager.read_image(subscriber)
        except Exception as e:
            rospy.loginfo(f"Error reading image: {e}")
            return False, np.array([])

    def start_video_stream(self) -> bool:
        """Starts the video stream from the drone's camera."""
        return self.sensor_manager.check_camera()

    # Helper Methods
    def _normalize_command(self, value: float, min: float = -1,
                           max: float = 1, prop: float = 100) -> float:
        """Normalizes command values to the range [-1, 1]."""
        return max(min(value / prop, max), min)

    def _execute_command(self, command: Callable, action_description: str
                         ) -> None:
        """
        Executes a drone command with error handling.

        :param command: The command to execute.
        :param action_description: Description of the action for logging.
        """
        try:
            command()
        except Exception as e:
            rospy.loginfo(f"Error during {action_description}: {e}")

    def _sensor_update_loop(self) -> None:
        """Continuously updates sensor data at the specified frequency."""
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update_sensors()
            rate.sleep()
