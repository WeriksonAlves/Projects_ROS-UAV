import os
from typing import Dict
from ..ros.DroneSensors import DroneSensors


class DroneSensorManager:
    """
    Manages the processing and access to the drone's sensor data.
    """

    def __init__(self, sensors: DroneSensors, ip_address: str = "192.168.0.202"
                 ) -> None:
        """
        Initializes the sensor manager with the given sensors and IP address.

        :param sensors: DroneSensors object.
        :param ip_address: IP address of the drone.
        """
        self.sensors = sensors
        self.sensor_data = self._initialize_sensor_data()
        self.status_flags = self._initialize_status_flags()
        self.ip_address = ip_address

    def _initialize_sensor_data(self) -> Dict[str, object]:
        """Initializes the sensor data dictionary with default values."""
        return {
            'altitude': 0.0, 'attitude': [0.0] * 3, 'battery': 100,
            'camera': None, 'gps': [0.0] * 3, 'image': None,
            'orientation': [0.0] * 3, 'position': [0.0] * 3,
            'speed_angular': [0.0] * 3, 'speed_linear': [0.0] * 3,
            'state': "unknown", 'wifi_signal': 0.0
        }

    def _initialize_status_flags(self) -> Dict[str, bool]:
        """Initializes the drone's status flags."""
        return {
            'connected': False, 'battery_full': False, 'battery_low': False,
            'battery_critical': False, 'emergency': False, 'hovering': False,
            'landed': True, 'moving': False, 'image_on': False,
            'recording': False, 'video_on': False, 'stabilized': False,
            'automatic': False, 'manual': False, 'gps_fixed': False,
            'gps_updated': False, 'temperature_updated': False,
            'pressure_updated': False, 'state_updated': False
        }

    def update_sensor_data(self) -> None:
        """Updates sensor data."""
        new_data = self.sensors.get_processed_sensor_data()
        self.sensor_data.update(new_data)

    def get_sensor_data(self) -> Dict[str, object]:
        """Returns the most recent sensor data."""
        return self.sensor_data

    def check_connection(self) -> None:
        """Check that the drone is connected to the network."""
        self.status_flags['connected'] = os.system(
            f"ping -c 1 {self.ip_address}") == 0

    def is_emergency(self) -> bool:
        """Check if the drone is in emergency status."""
        return self.status_flags['emergency']

    def is_hovering(self) -> bool:
        """Check that the drone is in hovering mode."""
        return self.status_flags['hovering']

    def is_landed(self) -> bool:
        """Check that the drone is on the ground."""
        return self.status_flags['landed']

    def get_battery_level(self) -> int:
        """Returns the drone's battery level."""
        return self.sensor_data['battery']
