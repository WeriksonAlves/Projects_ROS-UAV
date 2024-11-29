from py_uav.Bebop2 import Bebop2


class Bebop2DroneManager:
    """
    A class to manage and execute Bebop2 drone operations. Encapsulates
    connection, sensor updates, state management, and flight logic.
    """

    def __init__(self, drone_type: str = 'bebop2') -> None:
        """
        Initializes the Bebop2DroneManager instance.

        :param drone_type: The type of drone being controlled. Default is
                            'bebop2'.
        """
        self.drone = Bebop2(drone_type=drone_type)

    def connect_to_drone(self) -> bool:
        """
        Connects to the Bebop2 drone.

        :return: True if the connection is successful, False otherwise.
        """
        print("Connecting to Bebop2 drone...")
        if self.drone.check_connection():
            print("Successfully connected to the drone.")
            return True
        print("Failed to connect to the Bebop2 drone. Please check the",
              " connection and try again.")
        return False

    def print_battery_level(self) -> None:
        """
        Prints the current battery level of the drone.
        """
        battery_level = self.drone.sensor_manager.sensor_data.get(
            'battery_level', 'Unknown'
        )
        print(f"Battery Level: {battery_level}")

    def start_video_stream(self) -> None:
        """
        Starts the video stream from the drone and allows it to stabilize.
        """
        print("Starting video stream...")
        self.drone.start_video_stream()
        self.drone.smart_sleep(2)  # Wait for the video stream to stabilize

    def update_drone_sensors(self) -> None:
        """
        Updates the drone's state to retrieve the latest sensor data.
        """
        print("Requesting state update...")
        self.drone.update_sensors()

    def configure_indoor_flight(self) -> None:
        """
        Configures the drone's state for safe indoor flying.
        """
        print("Setting indoor flight parameters...")
        self.drone.state_manager.set_max_tilt(5)
        self.drone.state_manager.set_max_vertical_speed(1)

    def perform_indoor_flight(self) -> None:
        """
        Executes a sequence of flight commands for indoor operations.
        """
        print("Flying direct: Slow move for indoors")
        flight_commands = [
            (0, 0, 10, 0, 5),  # Hover up
            (10, 0, 0, 0, 5),  # Move forward
            (0, 10, 0, 0, 5),  # Move right
            (0, 0, 0, 0, 10)   # Hover in place
        ]

        for roll, pitch, vertical, yaw, duration in flight_commands:
            self.drone.fly_direct(roll, pitch, vertical, yaw,
                                  duration=duration)
            self.drone.smart_sleep(1)

    def land_drone(self) -> None:
        """
        Lands the drone safely. If already landed, executes an emergency stop.
        """
        print("Landing the drone...")
        self.drone.land()

    def run_experiment(self) -> None:
        """
        Executes the full experimental procedure with the Bebop2 drone.
        """
        if not self.connect_to_drone():
            return

        self.print_battery_level()
        self.start_video_stream()
        self.update_drone_sensors()

        self.drone.takeoff()
        self.configure_indoor_flight()
        self.perform_indoor_flight()

        self.land_drone()

        print("Experiment complete.")
        self.print_battery_level()


if __name__ == "__main__":
    drone_manager = Bebop2DroneManager()
    drone_manager.run_experiment()
