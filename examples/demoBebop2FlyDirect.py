#!/usr/bin/env python3

from rospy_uav.rospy_uav.Bebop2 import Bebop2


class demoBebop2FlyDirect:
    """
    A class to manage and execute a predefined flight pattern for a Bebop2
    drone.
    """

    def __init__(self, drone: Bebop2) -> None:
        """
        Initializes the DroneActionManager with a Bebop2 instance.

        :param drone: An instance of the Bebop2 drone.
        """
        self.drone = drone

    def execute_flight_pattern(self) -> None:
        """
        Executes a predefined flight pattern suitable for indoor operation.
        """
        print("Executing flight pattern...")
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


def main():
    """
    Main function to control the drone operations.
    """
    drone_type = 'gazebo'  # Change to 'bebop2' for real drone usage
    ip_address = '192.168.0.202'  # Use appropriate IP for the real drone

    # Initialize the drone
    bebop = Bebop2(drone_type=drone_type, ip_address=ip_address)

    # Create the drone action manager
    action_manager = demoBebop2FlyDirect(bebop)

    # Connect to the drone
    if drone_type == 'bebop2':
        print("Connecting to the drone...")
        if not bebop.check_connection():
            print("Connection failed. Please check the connection.")
            return
        print("Connection successful.")

    # Display battery status
    battery_level = bebop.sensor_manager.sensor_data.get(
        "battery_level", "Unknown"
    )
    print(f"Battery Level: {battery_level}%")

    # Start video stream
    print("Initializing video stream...")
    bebop.camera_on()
    bebop.smart_sleep(1)

    # Execute flight pattern
    print("Executing flight pattern...")
    bebop.takeoff()
    bebop.smart_sleep(2)

    action_manager.execute_flight_pattern()

    bebop.land()

    # Display battery status
    battery_level = bebop.sensor_manager.sensor_data.get(
        "battery_level", "Unknown"
    )
    print(f"Battery Level: {battery_level}%")


if __name__ == "__main__":
    main()
