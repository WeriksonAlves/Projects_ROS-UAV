#!/usr/bin/env python3

from rospy_uav.rospy_uav.Bebop2 import Bebop2
from typing import List


class FlightCommand:
    """
    Represents a single flight command for the drone.
    """

    def __init__(self, linear_x: int, linear_y: int, linear_z: int,
                 angular_z: int, duration: int) -> None:
        """
        Initializes a flight command.

        :param linear_x: Forward/backward speed.
        :param linear_y: Left/right speed.
        :param linear_z: Up/down speed.
        :param angular_z: Rotational speed.
        :param duration: Duration to execute the command.
        """
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.linear_z = linear_z
        self.angular_z = angular_z
        self.duration = duration


class FlightPattern:
    """
    Manages predefined flight patterns for the drone.
    """

    @staticmethod
    def get_indoor_flight_pattern() -> List[FlightCommand]:
        """
        Returns a list of flight commands for an indoor flight pattern.

        :return: List of FlightCommand objects.
        """
        return [
            FlightCommand(0, 0, 50, 0, 3),  # Hover up
            FlightCommand(25, 0, 0, 0, 2),   # Move forward
            FlightCommand(-25, 25, 0, 0, 2),   # Move right
            FlightCommand(0, -25, 0, 0, 2),  # Move back and left
            FlightCommand(0, 0, 0, 50, 2),  # Rotate
            FlightCommand(0, 0, 0, -50, 2),  # Rotate
            FlightCommand(0, 0, -50, 0, 3),  # Hover down
        ]


class DroneActionManager:
    """
    Manages drone operations, including executing flight patterns.
    """

    def __init__(self, drone: Bebop2) -> None:
        """
        Initializes the DroneActionManager with a Bebop2 instance.

        :param drone: An instance of the Bebop2 drone.
        """
        self.drone = drone

    def execute_flight_pattern(self, pattern: List[FlightCommand]) -> None:
        """
        Executes a sequence of flight commands.

        :param pattern: List of FlightCommand objects.
        """
        print("Executing flight pattern...")
        for command in pattern:
            self.drone.fly_direct(
                linear_x=command.linear_x,
                linear_y=command.linear_y,
                linear_z=command.linear_z,
                angular_z=command.angular_z,
                duration=command.duration
            )
            self.drone.smart_sleep(command.duration)


def main() -> None:
    """
    Main function to control the drone operations.
    """
    drone_type = 'gazebo'  # Change to 'bebop2' for real drone usage
    ip_address = '192.168.0.202'  # Use appropriate IP for the real drone

    # Initialize the drone
    bebop = Bebop2(drone_type=drone_type, ip_address=ip_address)

    # Create the drone action manager
    action_manager = DroneActionManager(bebop)

    # Connect to the drone
    if drone_type == 'bebop2':
        print("Connecting to the drone...")
        if not bebop.check_connection():
            print("Connection failed. Please check the connection.")
            return
        print("Connection successful.")

    # Display battery status
    battery_level = bebop.sensor_manager.sensor_data.get("battery_level",
                                                         "Unknown")
    print(f"Battery Level: {battery_level}%")

    # Start video stream
    print("Initializing video stream...")
    bebop.camera_on()
    bebop.smart_sleep(1)

    # Execute flight pattern
    print("Starting takeoff...")
    bebop.takeoff()
    bebop.smart_sleep(2)

    flight_pattern = FlightPattern.get_indoor_flight_pattern()
    action_manager.execute_flight_pattern(flight_pattern)

    print("Landing...")
    bebop.land()

    # Display final battery status
    battery_level = bebop.sensor_manager.sensor_data.get("battery_level",
                                                         "Unknown")
    print(f"Final Battery Level: {battery_level}%")


if __name__ == "__main__":
    main()
