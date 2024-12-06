#!/usr/bin/env python3

from typing import List, Tuple
import matplotlib.pyplot as plt
import numpy as np
from rospy_uav.rospy_uav.Bebop2 import Bebop2
from rospy_uav.rospy_uav.utils.DrawGraphics import set_axes_equal


class DroneTrajectoryManager:
    """
    Manages the trajectory execution and visualization for a Bebop2 drone.
    """

    def __init__(self, drone_type: str = "bebop2",
                 ip_address: str = "192.168.0.202", trajectory: str = None
                 ) -> None:
        """
        Initializes the DroneTrajectoryManager instance.

        :param drone_type: Type of the drone (e.g., 'bebop2', 'gazebo').
        :param ip_address: IP address of the drone.
        :param trajectory: Predefined trajectory to execute ('cube', 'ellipse',
                            or 'lemniscate').
        """
        self.drone = Bebop2(drone_type=drone_type, ip_address=ip_address)
        self.trajectory = trajectory

    def connect_to_drone(self) -> bool:
        """
        Attempts to connect to the drone.

        :return: True if connection is successful, False otherwise.
        """
        print("Connecting to Bebop2 drone...")
        if self.drone.drone_type == "bebop2" and not self.connect_to_drone():
            print("Connection successful.")
            return True
        print("Failed to connect. Please check the connection.")
        return False

    def display_battery_status(self) -> None:
        """
        Prints the drone's current battery level.
        """
        battery_level = self.drone.sensor_manager.sensor_data.get(
            "battery_level", "Unknown"
        )
        print(f"Battery Level: {battery_level}%")

    def start_video_stream(self, stabilization_time: int = 2) -> None:
        """
        Starts the drone's video stream.

        :param stabilization_time: Time (in seconds) to stabilize the video
                                    stream.
        """
        print("Initializing video stream...")
        self.drone.camera_on()
        self.drone.smart_sleep(stabilization_time)

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

    def execute_trajectory(
            self,
            trajectory_points: List[Tuple[float, float, float, float, float]],
            sleep_time: float = 1.0) -> List[Tuple[float, float, float]]:
        """
        Executes a given trajectory and logs the drone's positions.

        :param trajectory_points: List of (x, y, z, yaw) points to follow.
        :param sleep_time: Time (in seconds) to pause between movements.
        :return: List of recorded (x, y, z) positions during the trajectory.
        """
        positions = []
        for x, y, z, yaw, power_level in trajectory_points:
            self.drone.move_relative(x, y, z, yaw, power_level)
            self.drone.smart_sleep(sleep_time)
            current_position = self.drone.sensor_manager.get_sensor_data()[
                "odometry"]["position"]
            positions.append(tuple(current_position))
            print(f"Current Position: x={current_position[0]:.3f}, "
                  f"y={current_position[1]:.3f}, z={current_position[2]:.3f}")
        return positions

    def generate_cube_trajectory(self
                                 ) -> List[Tuple[float, float, float, float]]:
        """
        Generates a predefined cube-shaped trajectory.

        :return: List of (x, y, z, yaw) points for the trajectory.
        """
        return [
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

    def generate_ellipse_trajectory(
            self, a: float = 2, b: float = 1, z: float = 3, points: int = 50
            ) -> List[Tuple[float, float, float, float]]:
        """
        Generates an elliptical trajectory.

        :param a: Semi-major axis of the ellipse.
        :param b: Semi-minor axis of the ellipse.
        :param z: Fixed height for the ellipse.
        :param points: Number of points to define the ellipse.
        :return: List of (x, y, z, yaw) points for the trajectory.
        """
        return [(a * np.cos(2 * np.pi * i / points), b * np.sin(
            2 * np.pi * i / points), z, 0, 0.25) for i in range(points)]

    def generate_lemniscate_trajectory(
            self, a: float = 2, b: float = 1, z: float = 3, points: int = 50
            ) -> List[Tuple[float, float, float, float]]:
        """
        Generates a lemniscate (figure-eight) trajectory.

        :param a: Horizontal scaling factor.
        :param b: Vertical scaling factor.
        :param z: Fixed height for the lemniscate.
        :param points: Number of points to define the lemniscate.
        :return: List of (x, y, z, yaw) points for the trajectory.
        """
        return [
            (a * np.sin(t), b * np.sin(2*t), z, 0, 0.25)
            for t in np.linspace(0, 2 * np.pi, points)
        ]

    def run_experiment(self) -> None:
        """
        Executes the complete trajectory experiment based on the specified
        trajectory type.
        """
        if self.drone.drone_type == "bebop2" and not self.connect_to_drone():
            return

        self.display_battery_status()
        self.start_video_stream()

        print("Taking off...")
        self.drone.takeoff()
        self.drone.smart_sleep(2)

        trajectory_functions = {
            "cube": self.generate_cube_trajectory,
            "ellipse": self.generate_ellipse_trajectory,
            "lemniscate": self.generate_lemniscate_trajectory,
        }

        if self.trajectory in trajectory_functions:
            trajectory_points = trajectory_functions[self.trajectory]()
            positions = self.execute_trajectory(trajectory_points)
            self.plot_trajectory(positions)
        else:
            print("Invalid trajectory specified. Please choose 'cube', "
                  "'ellipse', or 'lemniscate'.")

        self.drone.land()
        print("Experiment complete.")
        self.display_battery_status()


if __name__ == "__main__":
    # Specify the trajectory: 'cube', 'ellipse', or 'lemniscate'.
    manager = DroneTrajectoryManager(
        drone_type="gazebo", trajectory="lemniscate"
        )
    manager.run_experiment()
