"""
Purpose: Retrieves data from the Bebop2 drone's sensors, including GPS,
attitude, speed, and battery levels. Organizes this data into a structured
dictionary.
"""

import rospy
# Example message type; adapt as needed
from ..ros.DroneSensors import DroneSensors


class SensorManager:
    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes SensorManager for managing sensor data.

        :param drone_type: Specifies the type of drone.
        :param frequency: Frequency for command intervals in Hz (default: 30).
        """
        self.drone_type = drone_type
        self.frequency = frequency
        self.sensor_parser = DroneSensors(drone_type)
        self.last_command_time = rospy.get_time()
        self.command_interval = 1.0 / frequency

    def _is_time_to_command(self) -> bool:
        """
        Checks if enough time has passed since the last command.

        :return: True if the command interval has passed; False otherwise.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

