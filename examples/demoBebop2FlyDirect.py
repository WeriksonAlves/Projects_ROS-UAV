#!/usr/bin/env python3
from rospy_uav.rospy_uav.Bebop2 import Bebop2


class demoBebop2FlyDirect:
    """
    Manages and executes Bebop2 drone operations, including connection,
    sensor updates, and flight commands.
    """

    def __init__(self, drone_type: str = 'bebop2',
                 ip_address: str = '192.168.0.202') -> None:
        """
        Initializes the drone manager instance.

        :param drone_type: Type of drone being controlled.
        :param ip_address: IP address of the drone.
        """
        self.drone = Bebop2(drone_type=drone_type, ip_address=ip_address)

    def connect_to_drone(self) -> bool:
        """
        Attempts to connect to the drone.

        :return: True if connection is successful, otherwise False.
        """
        print("Connecting to Bebop2 drone...")
        if self.drone.check_connection():
            print("Connection successful.")
            return True
        print("Connection failed. Please check the connection.")
        return False

    def display_battery_status(self) -> None:
        """
        Displays the current battery level of the drone.
        """
        battery_level = self.drone.sensor_manager.sensor_data.get(
            "battery_level", "Unknown"
        )
        print(f"Battery Level: {battery_level}%")

    def start_video_stream(self, duration=2) -> None:
        """
        Starts the drone's video stream.
        """
        print("Initializing video stream...")
        self.drone.camera_on()
        self.drone.smart_sleep(duration)

    def execute_flight_pattern(self, duration=2) -> None:
        """
        Executes a predefined flight pattern for indoor operation.
        """
        print("Executing indoor flight pattern...")
        flight_commands = [
            (0, 0, 100, 0, 5),  # Hover up
            (75, 0, 0, 0, 5),   # Move forward
            (0, 50, 0, 0, 5),   # Move right
            (-75, -50, 0, 0, 5),  # Move back and left
            (0, 0, -100, 0, 5)  # Hover down
        ]

        for roll, pitch, vertical, yaw, duration in flight_commands:
            self.drone.fly_direct(
                linear_x=roll,
                linear_y=pitch,
                linear_z=vertical,
                angular_z=yaw,
                duration=duration
            )
            self.drone.smart_sleep(duration)

    def land(self) -> None:
        """
        Lands the drone safely.
        """
        print("Landing the drone...")
        self.drone.land()

    def run_experiment(self) -> None:
        """
        Runs the full flight experiment sequence.
        """
        if self.drone.drone_type == 'bebop2':
            if not self.connect_to_drone():
                return

        self.display_battery_status()
        self.start_video_stream()
        self.drone.takeoff()
        self.drone.smart_sleep(5)

        self.execute_flight_pattern()
        self.land()

        print("Experiment complete.")
        self.display_battery_status()


if __name__ == "__main__":
    drone_manager = demoBebop2FlyDirect(drone_type='gazebo')
    drone_manager.run_experiment()
