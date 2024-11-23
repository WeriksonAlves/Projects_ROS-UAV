"""
This module provides a class for managing the drone's sensor data, control
states, and camera. It uses the DroneCamera, DroneControl, and DroneSensors
classes to interact with the drone's camera, control commands, and sensor data,
respectively. The DroneSensorManager class provides methods for updating and
accessing sensor data, checking the drone's connection status, managing status
flags, and interacting with the camera.
"""

from ..ros.DroneCamera import DroneCamera
from ..ros.DroneControl import DroneControl
from ..ros.DroneSensors import DroneSensors
from typing import Dict, Tuple
import os
import numpy as np
import rospy


class DroneSensorManager:
    """
    Manages the processing and access to the drone's sensor data, control
    states, and camera.
    """

    def __init__(self, drone_type: str, frequency: int, ip_address: str,
                 main_dir: str) -> None:
        """
        Initializes the DroneSensorManager with the given drone type and
        configuration.

        :param drone_type: Type of the drone (e.g., 'bebop2').
        :param frequency: Sensor data update frequency (Hz).
        :param ip_address: IP address of the drone for network checks.
        :param main_dir: Base directory for saving images or logs.
        """
        self.drone_camera = DroneCamera(drone_type, main_dir, frequency)
        self.drone_control = DroneControl(drone_type, frequency)
        self.sensors = DroneSensors(drone_type, frequency)
        self.ip_address = ip_address
        self.main_dir = main_dir

        # Used for generating unique snapshot filenames
        self.snapshot_counter = 0
        self.sensor_data = self._create_initial_sensor_data()
        self.status_flags = self._create_initial_status_flags()

    # Initialization Methods

    @staticmethod
    def _create_initial_sensor_data() -> Dict[str, object]:
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
    def _create_initial_status_flags() -> Dict[str, bool]:
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

    # Sensor Management

    def update_sensor_data(self) -> None:
        """
        Updates the internal sensor data from the DroneSensors module.
        """
        new_data = self.sensors.get_processed_sensor_data()
        self.sensor_data.update(new_data)

    def get_sensor_data(self) -> Dict[str, object]:
        """
        Retrieves the most recent sensor data.

        :return: A dictionary containing the latest sensor readings.
        """
        return self.sensor_data

    def get_battery_level(self) -> int:
        """
        Returns the drone's current battery level.

        :return: Battery level percentage.
        """
        return self.sensor_data.get('battery_level', 0)

    def check_connection(self, signal_threshold: int = -40) -> bool:
        """
        Checks if the drone is connected to the network and the signal is
        strong enough.

        :param signal_threshold: Minimum acceptable Wi-Fi signal strength
                                    (default: -40 dBm).
        :return: True if connected and signal strength is above the threshold,
                    False otherwise.
        """
        connection_status = os.system(f"ping -c 1 {self.ip_address}") == 0
        wifi_signal = self.sensor_data.get('wifi_signal', -100)

        if connection_status and wifi_signal > signal_threshold:
            rospy.loginfo(f"Drone connected with signal: {wifi_signal} dBm")
            self.status_flags['connected'] = True
            return True

        rospy.logwarn(f"Connection failed or weak signal: {wifi_signal} dBm")
        self.status_flags['connected'] = False
        return False

    # Status Flags Management

    def change_status_flags(self, name: str, value: bool) -> None:
        """
        Updates a specific status flag for the drone.

        :param name: Name of the status flag.
        :param value: New value for the status flag.
        """
        self.status_flags[name] = value

    def is_emergency(self) -> bool:
        """
        Checks if the drone is in an emergency state.

        :return: True if in emergency state, False otherwise.
        """
        return self.status_flags.get('emergency', None)

    def is_hovering(self) -> bool:
        """
        Checks if the drone is currently hovering.

        :return: True if hovering, False otherwise.
        """
        return self.status_flags.get('hovering', None)

    def is_landed(self) -> bool:
        """
        Checks if the drone is currently on the ground.

        :return: True if landed, False otherwise.
        """
        return self.status_flags.get('landed', None)

    # Camera Management

    def check_camera(self) -> bool:
        """
        Verifies if the drone's camera is operational.

        :return: True if the camera is operational, False otherwise.
        """
        camera_status = self.drone_camera.open_camera
        self.sensor_data['camera_on'] = camera_status
        return camera_status

    def read_image(self, subscriber: str = 'compressed') -> Tuple[bool,
                                                                  np.ndarray]:
        """
        Retrieves an image from the drone's camera.

        :param subscriber: The image subscriber type (default: 'compressed').
        :return: Tuple containing a success flag and the image data.
        """
        flag = self.drone_camera.success_flags.get(subscriber, False)
        data = self.drone_camera.image_data.get(subscriber, None)
        return flag, data if data is not None else np.array([])

    def take_snapshot(self, frame: np.ndarray) -> None:
        """
        Captures and saves a snapshot from the drone's camera.

        :param frame: The image frame to save.
        """
        filename = self._generate_unique_snapshot_filename()
        self.drone_camera.capture_snapshot(frame, filename)

    def _generate_unique_snapshot_filename(self) -> str:
        """
        Generates a unique filename for saving snapshots.

        :return: Unique filename for the snapshot.
        """
        snapshot_dir = os.path.join(self.main_dir, 'images', 'snapshot')
        os.makedirs(snapshot_dir, exist_ok=True)  # Ensure directory exists

        filename = f"img_{self.snapshot_counter:04d}.png"
        self.snapshot_counter += 1
        return os.path.join(snapshot_dir, filename)
