from abc import ABC, abstractmethod


class RosCommunication(ABC):
    """
    Implementation model for classes whose purpose is to communicate with ROS
    both to read data and to write data.
    """

    @abstractmethod
    def __init__(self, drone_type: str, frequency: int = 30) -> None:
        """
        Initializes the Interface class.
        """
        self.drone_type = drone_type
        self.command_interval = 1 / frequency

        self._init_subscribers()
        self._init_publishers()

    @abstractmethod
    def _initialize_subscribers(self) -> None:
        """
        Initializes subscribers to read ROS data.
        """
        pass

    @abstractmethod
    def _initialize_publishers(self) -> None:
        """
        Initializes publishers for writing data to ROS.
        """
        pass
