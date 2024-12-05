#!/usr/bin/env python3

from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from rospy_uav.rospy_uav.Bebop2 import Bebop2


class DroneTrajectoryManager:
    """
    Manages the trajectory execution and visualization for a Bebop2 drone.
    """

    def __init__(self, drone_type: str = 'bebop2',
                 ip_address: str = '192.168.0.202', trajectory: str = None
                 ) -> None:
        """
        Initializes the drone trajectory manager.

        :param drone_type: Type of the drone (e.g., 'bebop2', 'gazebo').
        :param ip_address: IP address of the drone.
        :param trajectory: Predefined trajectory to execute ('cube' or
                            'ellipse').
        """
        self.drone = Bebop2(drone_type=drone_type, ip_address=ip_address)
        self.trajectory = trajectory

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

    def start_video_stream(self, duration: int = 2) -> None:
        """
        Starts the drone's video stream.

        :param duration: Time to allow the video stream to stabilize.
        """
        print("Initializing video stream...")
        self.drone.camera_on()
        self.drone.smart_sleep(duration)

    def plot_trajectory(self, trajectory_data: List[Tuple[float, float, float]]
                        ) -> None:
        """
        Plots the 3D trajectory of the drone.

        :param trajectory_data: List of (x, y, z) coordinates.
        """
        x, y, z = zip(*trajectory_data)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x, y, z)
        ax.grid()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    def execute_trajectory_cube(self) -> None:
        """
        Executes a predefined cube-shaped trajectory.
        """
        print("Executing cube trajectory...")
        cube_vertices = [
            (0, 0, 1, 0),  # Vertex 1
            (1, 0, 1, 0),  # Vertex 2
            (1, 1, 1, 0),  # Vertex 3
            (0, 1, 1, 0),  # Vertex 4
            (0, 0, 1, 0),  # Return to Vertex 1
            (0, 0, 2, 0),  # Vertex 5
            (1, 0, 2, 0),  # Vertex 6
            (1, 1, 2, 0),  # Vertex 7
            (0, 1, 2, 0),  # Vertex 8
            (0, 0, 2, 0),  # Return to Vertex 5
            (0, 0, 1, 0),  # Return to Vertex 1
        ]

        for x, y, z, yaw in cube_vertices:
            self.drone.move_relative(x, y, z, yaw)
            self.drone.smart_sleep(1)
            position = self.drone.sensor_manager.get_sensor_data()['position']
            print(
                f"Current pose: x={position[0]}, "
                f"y={position[1]}, z={position[2]}")

    def execute_trajectory_ellipse(self) -> None:
        """
        Executes a predefined elliptical trajectory.
        """
        print("Executing elliptical trajectory...")
        a, b, z = 2.5, 1.5, 2
        points = 100
        interval = 0.1

        trajectory_data = []
        for i in range(points):
            x = a * np.cos(2 * np.pi * i / points)
            y = b * np.sin(2 * np.pi * i / points)
            self.drone.move_relative(x, y, z, 0)
            self.drone.smart_sleep(interval)
            position = self.drone.sensor_manager.get_sensor_data()['position']
            trajectory_data.append((position[0], position[1], position[2]))

        self.plot_trajectory(trajectory_data)

    def run_experiment(self) -> None:
        """
        Executes the complete trajectory experiment.

        :return: None
        """
        if self.drone.drone_type == 'bebop':
            if not self.connect_to_drone():
                return

        self.display_battery_status()
        self.start_video_stream()
        self.drone.takeoff()
        self.drone.smart_sleep(2)

        if self.trajectory == 'cube':
            self.execute_trajectory_cube()
        elif self.trajectory == 'ellipse':
            self.execute_trajectory_ellipse()
        else:
            print(
                "Invalid trajectory specified. "
                "Please choose 'cube' or 'ellipse'."
            )

        self.drone.smart_sleep(2)
        self.drone.land()
        print("Experiment complete.")
        self.display_battery_status()


if __name__ == "__main__":
    # Replace 'cube' with 'ellipse' to run a different trajectory
    drone_manager = DroneTrajectoryManager(
        drone_type='gazebo', trajectory='cube'
    )
    drone_manager.run_experiment()
