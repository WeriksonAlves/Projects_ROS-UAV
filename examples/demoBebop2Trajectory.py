#!/usr/bin/env python3

from rospy_uav.rospy_uav.Bebop2 import Bebop2
from rospy_uav.rospy_uav.utils.DrawGraphics import DrawGraphics
from typing import List, Tuple, Callable
import numpy as np
import matplotlib.pyplot as plt


class TrajectoryGenerator:
    """
    Generates predefined trajectories for drone operations.
    """

    @staticmethod
    def generate_cube() -> List[Tuple[float, float, float, float, float]]:
        """
        Generates a cube-shaped trajectory: (x, y, z, yaw, power).
        """
        return [
            (0, 0, 1, 0, 0.5),  # Vertex 1
            (1, 0, 1, 0, 0.5),  # Vertex 2
            (1, 1, 1, 0, 0.5),  # Vertex 3
            (0, 1, 1, 0, 0.5),  # Vertex 4
            (0, 0, 1, 0, 0.5),  # Return to Vertex 1
            (0, 0, 2, 0, 0.5),  # Vertex 5
            (1, 0, 2, 0, 0.5),  # Vertex 6
            (1, 1, 2, 0, 0.5),  # Vertex 7
            (0, 1, 2, 0, 0.5),  # Vertex 8
            (0, 0, 2, 0, 0.25),  # Return to Vertex 5
            (0, 0, 1, 0, 0.25),  # Return to Vertex 1
        ]

    @staticmethod
    def generate_ellipse(a: float = 2, b: float = 1, z: float = 3,
                         points: int = 50, seq: int = 1) -> List[Tuple[
                             float, float, float, float, float]]:
        """
        Generates an elliptical trajectory.
        :param a: Semi-major axis length.
        :param b: Semi-minor axis length.
        :param z: Fixed altitude.
        :param points: Number of points to generate.
        :param seq: Sequence number for the ellipse.
        :return: List of trajectory points (x, y, z, yaw, power).
        """
        return [
            (a * np.cos(seq * t), b * np.sin(seq * t), z, 0, 0.25)
            for t in np.linspace(0, 2 * np.pi, points)
        ]

    @staticmethod
    def generate_lemniscate(a=2, b=1, z=3, points=50
                            ) -> List[Tuple[float, float, float, float, float]
                                      ]:
        """
        Generates a lemniscate (figure-eight) trajectory.
        :param a: Horizontal scaling factor.
        :param b: Vertical scaling factor.
        :param z: Fixed altitude.
        :param points: Number of points to generate.
        """
        return [
            (a * np.sin(t), b * np.sin(2 * t), z, 0, 0.25)
            for t in np.linspace(0, 2 * np.pi, points)
        ]


class DroneTrajectoryManager:
    """
    Manages drone trajectory execution.
    """

    def __init__(self, drone: Bebop2) -> None:
        """
        Initialize the DroneTrajectoryManager with a drone instance.
        :param drone: Instance of the Bebop2 drone.
        """
        self.drone = drone

    def execute_trajectory(self, trajectory: List[Tuple[
            float, float, float, float, float]]) -> List[Tuple[
                float, float, float]]:
        """
        Executes a given trajectory and records drone positions.
        :param trajectory: List of trajectory points (x, y, z, yaw, power).
        :return: List of recorded drone positions.
        """
        positions = []
        for x, y, z, yaw, power in trajectory:
            self.drone.move_relative(x, y, z, yaw, power)
            self.drone.smart_sleep(0.1)
            odometry = self.drone.sensor_manager.get_sensor_data().get(
                "odometry", {}
            )
            position = tuple(odometry.get("position", (0, 0, 0)))
            positions.append(position)
        return positions


def main():
    """
    Main function to execute the drone trajectory experiment.
    """
    # Configuration
    drone_type = 'gazebo'  # Change to 'bebop2' for real drone usage
    ip_address = '192.168.0.202'  # Replace with the real drone's IP address
    trajectory_type = "lemniscate"  # Options: 'cube', 'ellipse', 'lemniscate'

    # Initialize the drone
    bebop = Bebop2(drone_type=drone_type, ip_address=ip_address)
    trajectory_manager = DroneTrajectoryManager(bebop)

    # Connect to the drone
    if drone_type == 'bebop2' and not bebop.check_connection():
        print("Failed to connect to the drone. Please check the connection.")
        return
    print("Drone connected successfully.")

    # Display battery status
    battery_level = bebop.sensor_manager.sensor_data.get("battery_level",
                                                         "Unknown")
    print(f"Battery Level: {battery_level}%")

    # Start video streaming
    print("Starting video stream...")
    bebop.camera_on()
    bebop.smart_sleep(1)

    # Takeoff
    print("Taking off...")
    bebop.takeoff()
    bebop.smart_sleep(2)

    # Generate and execute the selected trajectory
    trajectory_map: dict[str, Callable[..., List[Tuple[
        float, float, float, float, float]]]] = {
        "cube": TrajectoryGenerator.generate_cube,
        "ellipse": TrajectoryGenerator.generate_ellipse,
        "lemniscate": TrajectoryGenerator.generate_lemniscate,
    }

    if trajectory_type in trajectory_map:
        trajectory_func = trajectory_map[trajectory_type]
        trajectory_points = trajectory_func()
        print(f"Executing {trajectory_type} trajectory...")
        recorded_positions = trajectory_manager.execute_trajectory(
            trajectory_points
        )
        DrawGraphics.plot_trajectory(recorded_positions)
    else:
        print(f"Invalid trajectory type: {trajectory_type}. Options: 'cube', "
              "'ellipse', 'lemniscate'.")

    # Land the drone
    print("Landing...")
    bebop.land()

    # Display final battery status
    battery_level = bebop.sensor_manager.sensor_data.get("battery_level",
                                                         "Unknown")
    print(f"Final Battery Level: {battery_level}%")

    # Show trajectory plot
    plt.show()


if __name__ == "__main__":
    main()
