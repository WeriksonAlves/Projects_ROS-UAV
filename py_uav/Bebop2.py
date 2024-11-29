#!/usr/bin/env python3

from .commandsandsensors.DroneCommandManager import DroneCommandManager
from .commandsandsensors.DroneSensorManager import DroneSensorManager
from .commandsandsensors.DroneSettingManager import DroneSettingManager
from .ros.DroneCamera import DroneCamera
from .ros.DroneControl import DroneControl
from .ros.DroneSensors import DroneSensors
import numpy as np
import os
import rospy
import threading
from typing import Callable, Tuple


class Bebop2:
    """
    Interface for controlling and managing the Bebop2 drone using ROS.
    Provides drone control, sensor management, and camera operations.
    """

    def __init__(self, drone_type: str, ip_address: str,
                 frequency: float = 30.0) -> None:
        """
        Initialize the Bebop2 class.

        :param drone_type: Type of the drone ('bebop2' or 'gazebo').
        :param ip_address: IP address of the drone.
        :param frequency: Frequency for sensor updates in Hz.
        """
        rospy.init_node("ROS_UAV", anonymous=True)

        self.drone_type = drone_type
        self.ip_address = ip_address
        self.frequency = frequency
        self.main_dir = os.path.dirname(os.path.abspath(__file__))

        # Initialize subsystems
        self.drone_camera = DroneCamera(drone_type, self.main_dir, frequency)
        self.drone_control = DroneControl(drone_type, frequency)
        self.drone_sensors = DroneSensors(drone_type, frequency)
        self.sensor_manager = DroneSensorManager(self.drone_camera,
                                                 self.drone_sensors)
        self.command_manager = DroneCommandManager(
            self.drone_camera, self.drone_control, self.sensor_manager
        )
        self.state_manager = DroneSettingManager(self.command_manager,
                                                 self.sensor_manager)

        # User-defined callback
        self.user_callback: Callable = None
        self.user_callback_args = ()

        # Start sensor update thread
        self.sensor_thread = threading.Thread(target=self._sensor_update_loop,
                                              daemon=True)
        self.sensor_thread.start()

    def set_user_sensor_callback(self, callback: Callable, *args) -> None:
        """
        Register a user-defined callback for sensor updates.

        :param callback: Function to execute as a callback.
        :param args: Arguments to pass to the callback.
        """
        self.user_callback = callback
        self.user_callback_args = args

    def trigger_callback(self) -> None:
        """Execute the user-defined callback with the provided arguments."""
        if self.user_callback:
            self.user_callback(*self.user_callback_args)

    # ---- Drone State and Utility Methods ----

    def smart_sleep(self, seconds: float) -> None:
        """
        Pause execution while allowing ROS processes to continue.

        :param seconds: Duration to sleep in seconds.
        """
        rospy.sleep(seconds)

    def update_sensors(self) -> None:
        """Update sensor data using the Sensor Manager."""
        self.sensor_manager.update_sensor_data()

    def check_connection(self) -> bool:
        """
        Check if the drone is connected to the network.

        :return: True if connected, False otherwise.
        """
        return self.sensor_manager.check_connection(self.ip_address)

    # ---- Drone Control Methods ----

    def takeoff(self) -> None:
        """Command the drone to take off."""
        self._execute_command(self.command_manager.takeoff, "takeoff")

    def safe_takeoff(self, timeout: float = 3.0) -> None:
        """
        Command the drone to take off safely within a timeout.

        :param timeout: Maximum time in seconds to attempt takeoff.
        """
        success = self._execute_command(
            lambda: self.command_manager.safe_takeoff(timeout), "safe takeoff"
        )
        if not success:
            self.emergency_land()

    def land(self) -> None:
        """Command the drone to land."""
        self._execute_command(self.command_manager.land, "landing")

    def safe_land(self, timeout: float = 3.0) -> None:
        """
        Command the drone to land safely within a timeout.

        :param timeout: Maximum time in seconds to attempt landing.
        """
        success = self._execute_command(
            lambda: self.command_manager.safe_land(timeout), "safe landing"
        )
        if not success:
            self.emergency_land()

    def emergency_land(self) -> None:
        """Perform an emergency stop and land."""
        self._execute_command(
            self.command_manager.emergency_stop, "emergency landing"
        )

    def is_landed(self) -> bool:
        """
        Checks if the drone is currently landed.

        :return: True if the drone is landed, False otherwise.
        """
        return self.sensor_manager.is_landed()

    def is_hovering(self) -> bool:
        """
        Checks if the drone is currently hovering.

        :return: True if the drone is hovering, False otherwise.
        """
        return self.sensor_manager.is_hovering()

    def is_emergency(self) -> bool:
        """
        Checks if the drone is in an emergency state.

        :return: True if the drone is in an emergency state, False otherwise.
        """
        return self.sensor_manager.is_emergency()

    def fly_direct(self, linear_x: float, linear_y: float, linear_z: float,
                   angular_z: float, duration: float) -> None:
        """
        Command the drone to fly directly with specified velocities.

        :param linear_x: Velocity in the x direction.
        :param linear_y: Velocity in the y direction.
        :param linear_z: Velocity in the z direction.
        :param angular_z: Rotational velocity around the z-axis.
        :param duration: Duration of movement in seconds.
        """
        normalized_velocities = [
            self._normalize_velocity(v) for v in [
                linear_x, linear_y, linear_z, angular_z
            ]
        ]
        self._execute_command(
            lambda: self.command_manager.fly_direct(
                *normalized_velocities, duration), "direct flight"
        )

    def flip(self, direction: str) -> None:
        """
        Command the drone to perform a flip in a specified direction.

        :param direction: Flip direction ('left', 'right', 'forward',
                            'backward').
        """
        self._execute_command(
            lambda: self.command_manager.flip(direction), f"flip {direction}"
        )

    def move_relative(self, delta_x: float, delta_y: float, delta_z: float,
                      delta_yaw: float, power: int) -> None:
        """
        Command the drone to move to a relative position.

        :param delta_x: Change in x direction.
        :param delta_y: Change in y direction.
        :param delta_z: Change in z direction.
        :param delta_yaw: Change in yaw angle.
        :param power: Movement power as a percentage (0 to 100).
        """
        self._execute_command(
            lambda: self.command_manager.move_relative(
                delta_x, delta_y, delta_z, delta_yaw, power
            ), "relative movement"
        )

    # ---- Camera Operations ----

    def release_camera(self) -> None:
        """Releases the camera resources."""
        self._execute_command(
            self.command_manager.release_camera, "release camera"
        )

    def adjust_camera_orientation(self, tilt: float, pan: float,
                                  pitch_comp: float = 0.0,
                                  yaw_comp: float = 0.0) -> None:
        """
        Adjust the camera's orientation.

        :param tilt: Camera tilt angle.
        :param pan: Camera pan angle.
        :param pitch_comp: Optional pitch compensation.
        :param yaw_comp: Optional yaw compensation.
        """
        self._execute_command(
            lambda: self.command_manager.adjust_camera_orientation(
                tilt - pitch_comp, pan - yaw_comp
            ), "adjust camera orientation"
        )

    def adjust_camera_exposure(self, exposure: float) -> None:
        """
        Adjust the camera's exposure setting.

        :param exposure: Exposure value between -3 and 3.
        """
        self._execute_command(
            lambda: self.command_manager.adjust_camera_exposure(exposure),
            "adjust camera exposure"
        )

    def take_snapshot(self) -> None:
        """Capture a snapshot from the camera."""
        ret, frame = self.read_image()
        if ret:
            self._execute_command(
                lambda: self.command_manager.save_snapshot(frame), "snapshot"
            )

    def read_image(self, subscriber: str = "compressed") -> Tuple[bool,
                                                                  np.ndarray]:
        """
        Read an image from the drone's camera.

        :param subscriber: Subscriber type for image data.
        :return: Tuple of success flag and image array.
        """
        try:
            return self.sensor_manager.read_image(subscriber)
        except Exception as e:
            rospy.loginfo(f"Error reading image: {e}")
            return False, np.array([])

    def camera_on(self) -> bool:
        """Check if the camera is operational."""
        if self.sensor_manager.is_camera_operational():
            rospy.loginfo("Camera is operational.")
            return True
        else:
            rospy.loginfo("Camera is not operational. Verify connection.")
            return False

    # ---- Helper Methods ----

    def _normalize_velocity(self, value: float, scale: int = 100,
                            min_val: float = -1.0, max_val: float = 1.0
                            ) -> float:
        """
        Normalize a velocity value within a specified range.

        :param value: Input velocity value.
        :param scale: Scaling factor.
        :param min_val: Minimum allowed value.
        :param max_val: Maximum allowed value.
        :return: Normalized velocity.
        """
        return max(min(value / scale, max_val), min_val)

    def _execute_command(self, command: Callable, action_description: str
                         ) -> bool:
        """
        Execute a drone command safely with error handling.

        :param command: The command to execute.
        :param action_description: Description of the action for logging.
        :return: True if the command succeeded, False otherwise.
        """
        try:
            command()
            rospy.loginfo(f"Successfully executed: {action_description}.")
            return True
        except Exception as e:
            rospy.loginfo(f"Error during {action_description}: {e}")
            return False

    def _sensor_update_loop(self) -> None:
        """
        Background loop for updating sensors at a fixed rate.
        """
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update_sensors()
            rate.sleep()
