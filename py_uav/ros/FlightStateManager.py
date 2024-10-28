"""
Purpose: This class will be responsible for tracking and managing flight state
information, such as flat trim, flight plan availability, or other important
state updates.

Topics (4):
    /bebop/states/ardrone3/PilotingState/FlatTrimChanged
    /bebop/states/ardrone3/PilotingState/NavigateHomeStateChanged
    /bebop/states/common/FlightPlanState/AvailabilityStateChanged
    /bebop/states/common/FlightPlanState/ComponentStateListChanged
"""

import rospy
from ..interfaces.RosCommunication import RosCommunication
from std_msgs.msg import Int32, String


class FlightStateManager(RosCommunication):
    """
    Class for managing flight state information on the drone.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes publishers and subscribers for managing flight state information.

        :param drone_type: The type of drone being used.
        :param frequency: The frequency at which to check flight state updates.
        """
        self.drone_type = drone_type
        self.command_interval = 1 / frequency
        self.last_command_time = rospy.get_time()

        self._initialize_publishers()
        self._initialize_subscribers()

        # Variables to hold the current flight states
        self.flat_trim = None
        self.navigate_home = None
        self.flight_plan_available = None
        self.flight_plan_components = None

    def _initialize_subscribers(self) -> None:
        """
        Initializes subscribers for flight state updates.
        """
        rospy.Subscriber(
            "/bebop/states/ardrone3/PilotingState/FlatTrimChanged",
            Int32, self._flat_trim_callback)
        rospy.Subscriber(
            "/bebop/states/ardrone3/PilotingState/NavigateHomeStateChanged",
            Int32, self._navigate_home_callback)
        rospy.Subscriber(
            "/bebop/states/common/FlightPlanState/AvailabilityStateChanged",
            Int32, self._flight_plan_availability_callback)
        rospy.Subscriber(
            "/bebop/states/common/FlightPlanState/ComponentStateListChanged",
            String, self._flight_plan_components_callback)

    def _initialize_publishers(self) -> None:
        """
        Initializes publishers for flight state information.
        """
        # No specific publishers required as per the current requirements
        pass

    def _flat_trim_callback(self, msg: Int32) -> None:
        """
        Callback for the flat trim state.

        :param msg: ROS message containing the flat trim state.
        """
        self.flat_trim = msg.data
        rospy.loginfo(f"Flat Trim State Updated: {self.flat_trim}")

    def _navigate_home_callback(self, msg: Int32) -> None:
        """
        Callback for the navigate home state.

        :param msg: ROS message containing the navigate home state.
        """
        self.navigate_home = msg.data
        rospy.loginfo(f"Navigate Home State Updated: {self.navigate_home}")

    def _flight_plan_availability_callback(self, msg: Int32) -> None:
        """
        Callback for the flight plan availability state.

        :param msg: ROS message containing the flight plan availability state.
        """
        self.flight_plan_available = msg.data
        rospy.loginfo(f"Flight Plan Availability Updated: {self.flight_plan_available}")

    def _flight_plan_components_callback(self, msg: String) -> None:
        """
        Callback for the flight plan components state.

        :param msg: ROS message containing the flight plan component states.
        """
        self.flight_plan_components = msg.data
        rospy.loginfo(f"Flight Plan Components Updated: {self.flight_plan_components}")

    def check_state_updates(self) -> None:
        """
        Checks and processes state updates based on frequency.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            # Process state information or update UI if needed
            rospy.loginfo("Checking state updates...")
            # Add additional processing if required

    def get_flight_state(self) -> dict:
        """
        Returns the current state information.

        :return: Dictionary with current flight state information.
        """
        return {
            "flat_trim": self.flat_trim,
            "navigate_home": self.navigate_home,
            "flight_plan_available": self.flight_plan_available,
            "flight_plan_components": self.flight_plan_components,
        }
