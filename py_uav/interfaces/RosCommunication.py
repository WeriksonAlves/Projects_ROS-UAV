from abc import ABC, abstractmethod


class RosCommunication(ABC):
    """
    Abstract base class for ROS communication, providing an interface for
    classes designed to interact with ROS, both for data publishing and
    subscription.

    :param drone_type: Type of the drone (e.g., "Bebop2").
    :param frequency: Frequency of command updates, determining the interval
                      between commands (default: 30 Hz).
    """

    def __init__(self, drone_type: str, frequency: int = 30) -> None:
        """
        Initializes ROS communication settings and triggers setup of
        publishers and subscribers.

        :param drone_type: Type of the drone (e.g., "Bebop2").
        :param frequency: Frequency of command updates in Hz (default: 30).
        """
        pass

    @abstractmethod
    def _initialize_subscribers(self) -> None:
        """
        Initializes ROS subscribers to handle incoming data. This method
        must be implemented in subclasses to define specific ROS topics
        for subscription.
        """
        pass

    @abstractmethod
    def _initialize_publishers(self) -> None:
        """
        Initializes ROS publishers for sending data to ROS. This method
        must be implemented in subclasses to define specific ROS topics
        for publishing.
        """
        pass
