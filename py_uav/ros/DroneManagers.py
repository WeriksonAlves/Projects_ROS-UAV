"""
Purpose:  Subscribe to these topics, store parameter details,
and provide methods to adjust or retrieve current parameter states dynamically.


Topics (4):
    /bebop/bebop_driver/parameter_descriptions
    /bebop/bebop_driver/parameter_updates
    /bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged
    /bebop/states/common/OverHeatState/OverHeatChanged
"""

import rospy
from ..interfaces.RosCommunication import RosCommunication
from dynamic_reconfigure.msg import ConfigDescription, Config
from bebop_msgs.msg import Ardrone3GPSStateNumberOfSatelliteChanged
from bebop_msgs.msg import CommonOverHeatStateOverHeatChanged


class ParameterManager(RosCommunication):
    """
    Manages drone parameter descriptions and updates for real-time adjustments.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize ParameterManager, setting up subscribers for parameter
        topics.

        :param drone_type: Type of the drone.
        :param frequency: Frequency of command checks.
        """
        super().__init__()
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = rospy.get_time()
        self.parameters = {}

        self._initialize_subscribers()
        self._initialize_publishers()

    def _initialize_subscribers(self) -> None:
        """
        Set up subscribers for drone parameter topics.
        """
        rospy.Subscriber(
            "/bebop/bebop_driver/parameter_descriptions", ConfigDescription,
            self._parameter_description_callback)
        rospy.Subscriber("/bebop/bebop_driver/parameter_updates", Config,
                         self._parameter_update_callback)

    def _initialize_publishers(self) -> None:
        """
        Placeholder for any publishers if required in the future.
        """
        pass

    def _parameter_description_callback(self, msg: ConfigDescription) -> None:
        """
        Callback to process parameter descriptions.

        :param msg: Parameter descriptions message.
        """
        # Store or update parameter descriptions
        self.parameters['descriptions'] = msg

    def _parameter_update_callback(self, msg: Config) -> None:
        """
        Callback to process parameter updates.

        :param msg: Parameter updates message.
        """
        # Store or update parameter values
        self.parameters['updates'] = msg

    def get_parameter_descriptions(self) -> ConfigDescription:
        """
        Retrieve the latest parameter descriptions.

        :return ConfigDescription: Latest parameter descriptions.
        """
        return self.parameters.get('descriptions', None)

    def get_parameter_updates(self) -> Config:
        """
        Retrieve the latest parameter updates.

        :return Config: Latest parameter updates.
        """
        return self.parameters.get('updates', None)


class GPSStateManager(RosCommunication):
    """
    Monitors the GPS state, specifically the number of connected satellites.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize GPSStateManager with the drone type and command frequency.

        :param drone_type: Type of the drone.
        :param frequency: Frequency of command checks.
        """
        super().__init__()
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = rospy.get_time()
        self.satellite_count = 0

        self._initialize_subscribers()
        self._initialize_publishers()

    def _initialize_subscribers(self) -> None:
        """
        Set up subscriber for GPS satellite count changes.
        """
        rospy.Subscriber(
            "/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged",
            Ardrone3GPSStateNumberOfSatelliteChanged, self._gps_state_callback)

    def _initialize_publishers(self) -> None:
        """
        Placeholder for any publishers if required in the future.
        """
        pass

    def _gps_state_callback(self,
                            msg: Ardrone3GPSStateNumberOfSatelliteChanged
                            ) -> None:
        """
        Callback to process the number of satellites.

        :param msg: GPS satellite count message.
        """
        # Update satellite count
        self.satellite_count = msg.numberOfSatellite

    def get_satellite_count(self) -> int:
        """
        Retrieve the current number of satellites.

        :return int: Current satellite count.
        """
        return self.satellite_count


class HealthMonitor(RosCommunication):
    """
    Monitors the drone's health state, specifically overheat warnings.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize HealthMonitor with the drone type and command frequency.

        :param drone_type: Type of the drone.
        :param frequency: Frequency of command checks.
        """
        super().__init__()
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = rospy.get_time()
        self.overheat_status = False

        self._initialize_subscribers()
        self._initialize_publishers()

    def _initialize_subscribers(self) -> None:
        """
        Set up subscriber for overheat state changes.
        """
        rospy.Subscriber(
            "/bebop/states/common/OverHeatState/OverHeatChanged",
            CommonOverHeatStateOverHeatChanged, self._overheat_state_callback)

    def _initialize_publishers(self) -> None:
        """
        Placeholder for any publishers if required in the future.
        """
        pass

    def _overheat_state_callback(self, msg: CommonOverHeatStateOverHeatChanged
                                 ) -> None:
        """
        Callback to process overheat status.

        :param msg: Overheat status message.
        """
        # Update overheat status
        self.overheat_status = msg.overheat

    def is_overheating(self) -> bool:
        """
        Retrieve the current overheat status.

        :return bool: True if overheating, False otherwise.
        """
        return self.overheat_status
