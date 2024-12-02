"""
ParameterManager: Manages dynamic parameter descriptions and updates.
GPSStateManager: Tracks GPS state for satellite count.
HealthMonitor: Monitors the drone's overheat status.

ROS Topics (4):
    - /bebop/bebop_driver/parameter_descriptions
    - /bebop/bebop_driver/parameter_updates
    - /bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged
    - /bebop/states/common/OverHeatState/OverHeatChanged
"""

import rospy
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

    _instance = None  # Instância singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(ParameterManager, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes ParameterManager with subscribers for parameter topics.

        :param drone_type: Specifies the type of drone.
        :param frequency: Frequency for command intervals in Hz (default: 30).
        """
        if hasattr(self, '_initialized') and self._initialized:
            return

        super().__init__(drone_type, frequency)
        self.last_command_time = rospy.get_time()
        self.parameters = {}
        self._initialize_subscribers()

        self._initialized = True

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

    def _initialize_publishers(self) -> None:
        return super()._initialize_publishers()

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

    def _parameter_description_callback(self, msg: ConfigDescription) -> None:
        """Callback to handle parameter descriptions."""
        if self._is_time_to_command():
            self.parameters['descriptions'] = msg

    def _parameter_update_callback(self, msg: Config) -> None:
        """Callback to handle parameter updates."""
        if self._is_time_to_command():
            self.parameters['updates'] = msg

    def get_parameter_descriptions(self) -> ConfigDescription:
        """Retrieves the latest parameter descriptions."""
        return self.parameters.get('descriptions')

    def get_parameter_updates(self) -> Config:
        """Retrieves the latest parameter updates."""
        return self.parameters.get('updates')


class GPSStateManager(RosCommunication):
    """
    Manages GPS state by monitoring the number of connected satellites.
    """

    _instance = None  # Instância singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(GPSStateManager, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes GPSStateManager with a subscriber for GPS satellite count.

        :param drone_type: Specifies the type of drone.
        :param frequency: Frequency for command intervals in Hz (default: 30).
        """
        if hasattr(self, '_initialized') and self._initialized:
            return

        super().__init__(drone_type, frequency)
        self.last_command_time = rospy.get_time()
        self.satellite_count = 0
        self._initialize_subscribers()

        self._initialized = True

    def _initialize_subscribers(self) -> None:
        """Sets up subscriber for GPS satellite count updates."""
        rospy.Subscriber(
            "/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged",
            Ardrone3GPSStateNumberOfSatelliteChanged, self._gps_state_callback
        )

    def _initialize_publishers(self) -> None:
        return super()._initialize_publishers()

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

    def _gps_state_callback(self,
                            msg: Ardrone3GPSStateNumberOfSatelliteChanged
                            ) -> None:
        """Callback to handle GPS satellite count updates."""
        if self._is_time_to_command():
            self.satellite_count = msg.numberOfSatellite

    def get_satellite_count(self) -> int:
        """Retrieves the current number of satellites."""
        return self.satellite_count


class HealthMonitor(RosCommunication):
    """
    Monitors the drone's health state, specifically tracking overheat status.
    """

    _instance = None  # Instância singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(HealthMonitor, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes HealthMonitor with a subscriber for overheat state updates.

        :param drone_type: Specifies the type of drone.
        :param frequency: Frequency for command intervals in Hz (default: 30).
        """
        if hasattr(self, '_initialized') and self._initialized:
            return

        super().__init__(drone_type, frequency)
        self.overheat_status = False
        self._initialize_subscribers()

        self._initialized = True

    def _initialize_subscribers(self) -> None:
        """Sets up subscriber for overheat state updates."""
        rospy.Subscriber(
            "/bebop/states/common/OverHeatState/OverHeatChanged",
            CommonOverHeatStateOverHeatChanged, self._overheat_state_callback
        )

    def _initialize_publishers(self) -> None:
        return super()._initialize_publishers()

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

    def _overheat_state_callback(self, msg: CommonOverHeatStateOverHeatChanged
                                 ) -> None:
        """Callback to handle overheat status updates."""
        if self._is_time_to_command():
            self.overheat_status = msg.overheat

    def is_overheating(self) -> bool:
        """Checks if the drone is currently overheating."""
        return self.overheat_status
