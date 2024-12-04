"""
This module provides a DroneSensorManager class that acts as a facade for
managing the drone's sensors, control states, and camera. It simplifies
interaction with DroneCamera, DroneControl, and DroneSensors, providing
unified access for handling drone operations.
"""

from ..ros.DroneCamera import DroneCamera
from ..ros.DroneSensors import DroneSensors
from typing import Dict, Tuple
import numpy as np
import os
import rospy


class DroneSensorManager:
    """
    Facade for managing the drone's sensor data, control states, and camera.
    """

    def __init__(self, drone_camera: DroneCamera, drone_sensors: DroneSensors,
                 show_log: bool = False) -> None:
        """
        Initialize DroneSensorManager with the specified drone type and
        configuration.

        :param drone_camera: DroneCamera object for handling camera operations.
        :param drone_sensors: DroneSensors object for handling sensor data.
        """
        self.drone_camera = drone_camera
        self.drone_sensors = drone_sensors
        self.show_log = show_log

        self.sensor_data = self._initialize_sensor_data()
        self.status_flags = self._initialize_status_flags()

    # Initialization Methods

    @staticmethod
    def _initialize_sensor_data() -> Dict[str, object]:
        """
        Factory method to initialize sensor data with default values.

        :return: Dictionary of sensor data with default values.
        """
        return {
            'altitude': 0.0,
            'attitude': [0.0] * 3,
            'battery_level': 100,
            'camera': None,
            'flying_state': None,
            'gps_position': [0.0] * 3,
            'ground_truth': None,
            'image': None,
            'odometry': None,
            'position': [0.0] * 3,
            'speed_linear': [0.0] * 3,
            'state': "unknown",
            'wifi_signal': 0.0
        }

    @staticmethod
    def _initialize_status_flags() -> Dict[str, bool]:
        """
        Factory method to initialize status flags with default values.

        :return: Dictionary of status flags with default values.
        """
        return {
            'automatic': False,
            'battery_critical': False,
            'battery_full': False,
            'battery_low': False,
            'camera_on': False,
            'connected': False,
            'emergency': False,
            'gps_fixed': False,
            'gps_updated': False,
            'hovering': False,
            'landed': True,
            'manual': False,
            'moving': False,
            'pressure_updated': False,
            'recording': False,
            'stabilized': False,
            'state_updated': False,
            'temperature_updated': False,
            'video_on': False
        }

    # Sensor Data Management

    def update_sensor_data(self) -> None:
        """
        Update the internal sensor data from the DroneSensors module.
        """
        try:
            new_data = self.drone_sensors.get_processed_sensor_data()
            self.sensor_data.update(new_data)
        except Exception as e:
            rospy.logerr(f"Failed to update sensor data: {e}")

    def get_sensor_data(self) -> Dict[str, object]:
        """
        Retrieve the most recent sensor data.

        :return: A dictionary containing the latest sensor readings.
        """
        return self.sensor_data

    def get_battery_level(self) -> int:
        """
        Get the current battery level of the drone.

        :return: Battery level as a percentage.
        """
        return self.sensor_data.get('battery_level', 0)

    def check_connection(self, ip_address: str, signal_threshold: int = -40
                         ) -> bool:
        """
        Check the drone's connectivity and signal strength.

        :param ip_address: IP address of the drone.
        :param signal_threshold: Minimum acceptable Wi-Fi signal strength
                                    (default: -40 dBm).
        :return: True if connected and signal strength is above the threshold,
                    False otherwise.
        """
        connection_status = os.system(f"ping -c 1 {ip_address}") == 0
        wifi_signal = self.sensor_data.get('wifi_signal', -100)

        if connection_status and wifi_signal > signal_threshold:
            if self.show_log:
                rospy.loginfo(
                    f"Drone connected with signal strength: {wifi_signal} dBm")
            self.status_flags['connected'] = True
            return True

        rospy.logwarn(f"Connection failed or weak signal: {wifi_signal} dBm")
        self.status_flags['connected'] = False
        return False

    # Status Flag Management

    def update_status_flag(self, name: str, value: bool) -> None:
        """
        Update a specific status flag for the drone.

        :param name: The name of the status flag.
        :param value: The new value for the status flag.
        """
        if name in self.status_flags:
            self.status_flags[name] = value
        else:
            rospy.logwarn(f"Status flag '{name}' does not exist.")

    def is_emergency(self) -> bool:
        """
        Check if the drone is in an emergency state.

        :return: True if the drone is in an emergency state, None otherwise.
        """
        return self.status_flags.get('emergency', None)

    def is_hovering(self) -> bool:
        """
        Check if the drone is currently hovering.

        :return: True if hovering, None otherwise.
        """
        return self.status_flags.get('hovering', None)

    def is_landed(self) -> bool:
        """
        Check if the drone is currently landed.

        :return: True if landed, None otherwise.
        """
        return self.status_flags.get('landed', None)

    def is_moving(self) -> bool:
        """
        Check if the drone is currently moving.

        :return: True if moving, None otherwise.
        """
        return self.status_flags.get('moving', None)

    # Camera Management

    def is_camera_operational(self) -> bool:
        """
        Verify if the drone's camera is operational.

        :return: True if the camera is operational, False otherwise.
        """
        camera_status = self.drone_camera.open_camera
        self.sensor_data['camera_on'] = camera_status
        return camera_status

    def read_image(self, subscriber: str = 'compressed') -> Tuple[bool,
                                                                  np.ndarray]:
        """
        Read an image from the drone's camera.

        :param subscriber: The type of image subscriber (default:
                            'compressed').
        :return: Tuple containing a success flag and the image data.
        """
        flag = self.drone_camera.success_flags.get(subscriber, False)
        data = self.drone_camera.image_data.get(subscriber, None)
        return flag, data if data is not None else None
