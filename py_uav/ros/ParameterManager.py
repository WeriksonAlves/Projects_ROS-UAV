"""
Purpose: Responsibilities: Subscribe to these topics, store parameter details,
and provide methods to adjust or retrieve current parameter states dynamically.


Topics (2):
    /bebop/bebop_driver/parameter_descriptions
    /bebop/bebop_driver/parameter_updates
"""

import rospy
from ..interfaces.RosCommunication import RosCommunication
from dynamic_reconfigure.msg import ConfigDescription, Config


class ParameterManager(RosCommunication):
    """
    
    """

    def __init(self, drone_type: str, frequency: int = 30):
        """
        
        """
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = rospy.get_time()
        self.parameters = {}

        self._initialize_subscribers()
        self._initialize_publishers()

    def _initialize_subscribers(self) -> None:
        """
        
        """
        rospy.Subscriber(
            "/bebop/bebop_driver/parameter_descriptions",
            ConfigDescription, self._parameter_description_callback)
        rospy.Subscriber(
            "/bebop/bebop_driver/parameter_updates",
            Config, self._parameter_update_callback)
    
    def _initialize_publishers(self) -> None:
        """
        
        """
        pass

