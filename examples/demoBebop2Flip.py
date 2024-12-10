#!/usr/bin/env python3

from rospy_uav.rospy_uav import Bebop2


class DroneFlipManager:
    """
    Manages the execution of flip maneuvers for a Bebop2 drone.
    """

    def __init__(self, drone: Bebop2) -> None:
        """
        Initializes the DroneFlipManager with a Bebop2 instance.

        :param drone: Bebop2 instance.
        """
        self.drone = drone

    def execute_flip_maneuvers(self) -> None:
        """
        Executes a series of flip maneuvers.
        """
        flip_list = ['right', 'left', 'forward', 'backward']
        for flip in flip_list:
            self.drone.flip(flip)
            self.drone.smart_sleep(2)


def main() -> None:
    """
    Main function to control the drone operations.
    """
    drone_type = 'gazebo'  # Change to 'bebop2' for real drone usage
    ip_address = '192.168.0.202'  # Use appropriate IP for the real drone

    # Initialize the drone
    bebop = Bebop2(drone_type=drone_type, ip_address=ip_address)

    # Create the drone flip manager
    flip_manager = DroneFlipManager(bebop)

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

    # Execute flip maneuvers
    print("Starting takeoff...")
    bebop.takeoff()
    bebop.smart_sleep(2)

    flip_manager.execute_flip_maneuvers()

    print("Landing...")
    bebop.land()

    # Display final battery status
    battery_level = bebop.sensor_manager.sensor_data.get("battery_level",
                                                         "Unknown")
    print(f"Final Battery Level: {battery_level}%")


if __name__ == "__main__":
    main()
