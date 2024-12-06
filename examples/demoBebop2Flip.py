#!/usr/bin/env python3

from typing import List, Tuple
import matplotlib.pyplot as plt
from rospy_uav.rospy_uav.Bebop2 import Bebop2
from rospy_uav.rospy_uav.utils.DrawGraphics import set_axes_equal


class demoBebop2Flip:
    """
    Exemplifies the Bebop2 drone's flip maneuver capabilities.
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

    def plot_trajectory(self, trajectory_data: List[Tuple[float, float, float]]
                        ) -> None:
        """
        Plots the 3D trajectory of the drone.

        :param trajectory_data: List of (x, y, z) coordinates.
        """
        x, y, z = zip(*trajectory_data)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(x, y, z, label="Trajectory")
        ax.set_title("Drone Trajectory")
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_zlabel("Z (meters)")
        ax.legend()
        ax = set_axes_equal(ax)
        plt.grid()
        plt.show()

    def start_video_stream(self, duration=2) -> None:
        """
        Starts the drone's video stream.
        """
        print("Initializing video stream...")
        self.drone.camera_on()
        self.drone.smart_sleep(duration)

    def command_list(self) -> None:
        """
        Defines the list of flip maneuvers to execute
        """
        flip_list = ['right', 'left', 'forward', 'backward']
        for flip in flip_list:
            self.drone.flip(flip)
            self.drone.smart_sleep(2)

    def run_experiment(self) -> None:
        """
        Executes the flip maneuver experiment.
        """
        if self.drone.drone_type == "bebop2" and not self.connect_to_drone():
            return

        self.display_battery_status()
        self.start_video_stream()

        print("Taking off...")
        self.drone.takeoff()
        self.drone.smart_sleep(2)

        self.display_battery_status()
        self.start_video_stream()
        self.command_list()

        self.drone.land()
        print("Experiment complete.")
        self.display_battery_status()


if __name__ == "__main__":
    drone_manager = demoBebop2Flip(drone_type='gazebo')
    drone_manager.run_experiment()
