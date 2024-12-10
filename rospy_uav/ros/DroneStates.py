"""
FlightStateManager Module: Manages flight state information for the Bebop
drone, such as flat trim, navigate home state, and flight plan availability.

ROS Topics (4):
    - /bebop/states/ardrone3/PilotingState/FlatTrimChanged
    - /bebop/states/ardrone3/PilotingState/NavigateHomeStateChanged
    - /bebop/states/common/FlightPlanState/AvailabilityStateChanged
    - /bebop/states/common/FlightPlanState/ComponentStateListChanged
"""

import rospy
from ..interfaces.RosCommunication import RosCommunication
from std_msgs.msg import Int32, String


class FlightStateManager(RosCommunication):
    """
    Manages flight state information on the Bebop drone.
    Tracks flat trim, navigate home, flight plan availability, and component
    state.
    """

    _instance = None  # Instância singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(FlightStateManager, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes publishers and subscribers for managing flight state
        information.

        :param drone_type: The type of drone being used.
        :param frequency: Frequency for checking state updates (in Hz, default:
                          30 Hz).
        """
        if hasattr(self, '_initialized') and self._initialized:
            return

        super().__init__(drone_type, frequency)
        self.command_interval = 1 / frequency
        self.last_command_time = rospy.get_time()

        self._initialize_subscribers()
        self._initialize_state_variables()

        self._initialized = True

    def _initialize_subscribers(self) -> None:
        """Sets up ROS subscribers for flight state topics."""
        rospy.Subscriber(
            "/bebop/states/ardrone3/PilotingState/FlatTrimChanged",
            Int32, self._update_flat_trim)
        rospy.Subscriber(
            "/bebop/states/ardrone3/PilotingState/NavigateHomeStateChanged",
            Int32, self._update_navigate_home)
        rospy.Subscriber(
            "/bebop/states/common/FlightPlanState/AvailabilityStateChanged",
            Int32, self._update_flight_plan_availability)
        rospy.Subscriber(
            "/bebop/states/common/FlightPlanState/ComponentStateListChanged",
            String, self._update_flight_plan_components)

    def _initialize_publishers(self) -> None:
        return super()._initialize_publishers()

    def _initialize_state_variables(self) -> None:
        """Initializes state variables for flight state tracking."""
        self._flat_trim = None
        self._navigate_home = None
        self._flight_plan_available = None
        self._flight_plan_components = None

    # Callback methods for each state update

    def _update_flat_trim(self, msg: Int32) -> None:
        """
        Callback to update flat trim state.

        :param msg: ROS message containing flat trim state.
        """
        self._flat_trim = msg.data
        rospy.loginfo(f"Flat Trim State Updated: {self._flat_trim}")

    def _update_navigate_home(self, msg: Int32) -> None:
        """
        Callback to update navigate home state.

        :param msg: ROS message containing navigate home state.
        """
        self._navigate_home = msg.data
        rospy.loginfo(f"Navigate Home State Updated: {self._navigate_home}")

    def _update_flight_plan_availability(self, msg: Int32) -> None:
        """
        Callback to update flight plan availability state.

        :param msg: ROS message containing flight plan availability state.
        """
        self._flight_plan_available = msg.data
        rospy.loginfo(f"Flight Plan Availability Updated"
                      f": {self._flight_plan_available}")

    def _update_flight_plan_components(self, msg: String) -> None:
        """
        Callback to update flight plan components state.

        :param msg: ROS message containing flight plan component states.
        """
        self._flight_plan_components = msg.data
        rospy.loginfo(f"Flight Plan Components Updated"
                      f": {self._flight_plan_components}")

    def check_state_updates(self) -> None:
        """
        Checks and processes state updates based on the defined update
        interval.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            rospy.loginfo("Checking state updates...")
            # Additional processing or UI update if needed

    def get_flight_state(self) -> dict:
        """
        Retrieves the current flight state information.

        :return: Dictionary with the latest flight state information.
        """
        return {
            "flat_trim": self._flat_trim,
            "navigate_home": self._navigate_home,
            "flight_plan_available": self._flight_plan_available,
            "flight_plan_components": self._flight_plan_components,
        }
