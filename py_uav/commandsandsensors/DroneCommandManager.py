import rospy
from .DroneSensorManager import DroneSensorManager
from ..ros.DroneCamera import DroneCamera
from ..ros.DroneControl import DroneControl


class DroneCommandManager:
    def __init__(self, drone_type: str, frequency: int,
                 sensors: DroneSensorManager):
        self.drone_type = drone_type
        self.frequency = frequency
        self.sensors = sensors
        self.command_interval = 1.0 / frequency
        self.last_command_time = rospy.get_time()
        self.state_map = self._initialize_state_map()

        self._initialize_components()

        self.camera = DroneCamera(drone_type, frequency)
        self.control = DroneControl(drone_type, frequency)

    def _initialize_components(self) -> None:
        """Initializes all ROS-drone components."""
        self.camera = DroneCamera(self.drone_type, self.frequency)
        self.control = DroneControl(self.drone_type, self.frequency)
        self.gps = GPSStateManager(self.drone_type, self.frequency)
        self.health = HealthMonitor(self.drone_type, self.frequency)
        self.params = ParameterManager(self.drone_type, self.frequency)
        self.media = DroneMedia(self.drone_type, self.frequency)
        self.sensors = DroneSensors(self.drone_type, self.frequency)
        self.states = FlightStateManager(self.drone_type, self.frequency)

    @staticmethod
    def _initialize_state_map() -> dict:
        """Defines state mapping for the drone."""
        return {
            'E': 'emergency',
            'H': 'hovering',
            'L': 'landed',
            'l': 'landing',
            'M': 'moving',
            'T': 'takingoff'
        }

    def _is_time_to_command(self) -> bool:
        """
        Verifies if enough time has passed since the last command.

        :return: True if the command interval has passed; False otherwise.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    # Drone Control Methods

    # Camera Control Methods