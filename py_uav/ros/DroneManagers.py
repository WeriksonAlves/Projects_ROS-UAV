"""
ParameterManager: Handles dynamic parameter descriptions and updates.
GPSStateManager: Monitors GPS state for satellite count.
HealthMonitor: Monitors the drone's overheat status.

ROS Topics (4):
    - /bebop/bebop_driver/parameter_descriptions
    - /bebop/bebop_driver/parameter_updates
    - /bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged
    - /bebop/states/common/OverHeatState/OverHeatChanged
"""

import rospy
import time
from ..interfaces.RosCommunication import RosCommunication
from dynamic_reconfigure.msg import ConfigDescription, Config
from bebop_msgs.msg import (
    Ardrone3GPSStateNumberOfSatelliteChanged,
    CommonOverHeatStateOverHeatChanged
)


class ParameterManager(RosCommunication):
    """
    Manages drone parameters, handling descriptions and updates for real-time
    adjustments and retrieval.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes ParameterManager with subscribers for parameter topics.

        :param drone_type: Specifies the type of drone.
        :param frequency: Frequency for command intervals in Hz (default: 30).
        """
        super().__init__(drone_type, frequency)
        self.last_command_time = time.time()
        self.parameters = {}

    def _initialize_subscribers(self) -> None:
        """Sets up subscribers for parameter-related topics."""
        rospy.Subscriber(
            "/bebop/bebop_driver/parameter_descriptions",
            ConfigDescription, self._parameter_description_callback
        )
        rospy.Subscriber(
            "/bebop/bebop_driver/parameter_updates",
            Config, self._parameter_update_callback
        )

    def _parameter_description_callback(self, msg: ConfigDescription) -> None:
        """
        Callback to handle parameter descriptions.

        :param msg: ConfigDescription message containing parameter
                    descriptions.
        """
        self.parameters['descriptions'] = msg

    def _parameter_update_callback(self, msg: Config) -> None:
        """
        Callback to handle parameter updates.

        :param msg: Config message containing parameter updates.
        """
        self.parameters['updates'] = msg

    def get_parameter_descriptions(self) -> ConfigDescription:
        """
        Retrieves the latest parameter descriptions.

        :return: ConfigDescription object with the latest descriptions.
        """
        return self.parameters.get('descriptions')

    def get_parameter_updates(self) -> Config:
        """
        Retrieves the latest parameter updates.

        :return: Config object with the latest parameter updates.
        """
        return self.parameters.get('updates')


class GPSStateManager(RosCommunication):
    """
    Manages GPS state by monitoring the number of connected satellites.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes GPSStateManager with a subscriber for GPS satellite count.

        :param drone_type: Specifies the type of drone.
        :param frequency: Frequency for command intervals in Hz (default: 30).
        """
        super().__init__(drone_type, frequency)
        self.last_command_time = time.time()
        self.satellite_count = 0

    def _initialize_subscribers(self) -> None:
        """Sets up subscriber for GPS satellite count updates."""
        rospy.Subscriber(
            "/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged",
            Ardrone3GPSStateNumberOfSatelliteChanged, self._gps_state_callback
        )

    def _gps_state_callback(self,
                            msg: Ardrone3GPSStateNumberOfSatelliteChanged
                            ) -> None:
        """
        Callback to handle GPS satellite count updates.

        :param msg: Message with the number of satellites connected.
        """
        self.satellite_count = msg.numberOfSatellite

    def get_satellite_count(self) -> int:
        """
        Retrieves the current number of satellites.

        :return: Integer count of satellites.
        """
        return self.satellite_count


class HealthMonitor(RosCommunication):
    """
    Monitors the drone's health state, specifically tracking overheat status.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes HealthMonitor with a subscriber for overheat state updates.

        :param drone_type: Specifies the type of drone.
        :param frequency: Frequency for command intervals in Hz (default: 30).
        """
        super().__init__(drone_type, frequency)
        self.last_command_time = time.time()
        self.overheat_status = False

    def _initialize_subscribers(self) -> None:
        """Sets up subscriber for overheat state updates."""
        rospy.Subscriber(
            "/bebop/states/common/OverHeatState/OverHeatChanged",
            CommonOverHeatStateOverHeatChanged, self._overheat_state_callback
        )

    def _overheat_state_callback(self, msg: CommonOverHeatStateOverHeatChanged
                                 ) -> None:
        """
        Callback to handle overheat status updates.

        :param msg: Message indicating whether the drone is overheating.
        """
        self.overheat_status = msg.overheat

    def is_overheating(self) -> bool:
        """
        Checks if the drone is currently overheating.

        :return: True if overheating, False otherwise.
        """
        return self.overheat_status
