#!/usr/bin/env python3

from rospy_uav.rospy_uav.Bebop2 import Bebop2
from rospy_uav.rospy_uav.utils.DrawGraphics import set_axes_equal
from typing import List, Tuple
import matplotlib.pyplot as plt
import numpy as np


class demoBebop2Trajectory:

    @staticmethod
    def generate_cube() -> List[Tuple[float, float, float, float, float]]:
        """
        Generates a cube-shaped trajectory. (x, y, z, yaw, power_level)
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
    def generate_ellipse(a=2, b=1, z=3, points=50
                         ) -> List[Tuple[float, float, float, float, float]]:
        """
        Generates an elliptical trajectory.
        """
        return [(a * np.cos(2 * np.pi * i / points),
                 b * np.sin(2 * np.pi * i / points), z, 0, 0.25)
                for i in range(points)]

    @staticmethod
    def generate_lemniscate(a=2, b=1, z=3, points=50
                            ) -> List[Tuple[float, float, float, float, float]
                                      ]:
        """
        Generates a lemniscate (figure-eight) trajectory.
        """
        return [(a * np.sin(t), b * np.sin(2 * t), z, 0, 0.25)
                for t in np.linspace(0, 2 * np.pi, points)]




















### Voltar aqui




















class DroneController:
    """
    Manages the trajectory execution, video streaming, and data visualization for a Bebop2 drone.
    """

    def __init__(self, drone: Bebop2, trajectory_type: str = None) -> None:
        """
        Initialize the controller with a Bebop2 drone instance and trajectory type.
        """
        self.drone = drone
        self.trajectory_type = trajectory_type
        self.trajectory_map = {
            "cube": DroneTrajectory.generate_cube,
            "ellipse": DroneTrajectory.generate_ellipse,
            "lemniscate": DroneTrajectory.generate_lemniscate,
        }

    def connect_to_drone(self) -> bool:
        """
        Connects to the drone.
        """
        print("Connecting to the drone...")
        if self.drone.drone_type == "bebop2" and not self.drone.check_connection():
            print("Connection failed.")
            return False
        print("Connection successful.")
        return True

    def display_battery_status(self) -> None:
        """
        Displays the current battery level of the drone.
        """
        battery_level = self.drone.sensor_manager.sensor_data.get("battery_level", "Unknown")
        print(f"Battery Level: {battery_level}%")

    def start_video_stream(self, stabilization_time: int = 2) -> None:
        """
        Starts the drone's video stream.
        """
        print("Starting video stream...")
        self.drone.camera_on()
        self.drone.smart_sleep(stabilization_time)

    def execute_trajectory(self, trajectory_points: List[Tuple[float, float, float, float, float]]) -> List[Tuple[float, float, float]]:
        """
        Executes a trajectory and records the drone's positions.
        """
        positions = []
        for x, y, z, yaw, power in trajectory_points:
            self.drone.move_relative(x, y, z, yaw, power)
            self.drone.smart_sleep(1.0)
            position = tuple(self.drone.sensor_manager.get_sensor_data()["odometry"]["position"])
            print(f"Drone Position: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
            positions.append(position)
        return positions

    def plot_trajectory(self, trajectory_data: List[Tuple[float, float, float]]) -> None:
        """
        Plots the trajectory of the drone.
        """
        x, y, z = zip(*trajectory_data)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(x, y, z, label="Drone Path")
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_zlabel("Z (meters)")
        ax.legend()
        set_axes_equal(ax)
        plt.show()

    def run_experiment(self) -> None:
        """
        Executes the trajectory experiment.
        """
        if not self.connect_to_drone():
            return
        self.display_battery_status()
        self.start_video_stream()

        print("Taking off...")
        self.drone.takeoff()
        self.drone.smart_sleep(2)

        if self.trajectory_type in self.trajectory_map:
            trajectory_func = self.trajectory_map[self.trajectory_type]
            trajectory_points = trajectory_func()
            recorded_positions = self.execute_trajectory(trajectory_points)
            self.plot_trajectory(recorded_positions)
        else:
            print(f"Invalid trajectory type: {self.trajectory_type}. Choose from 'cube', 'ellipse', 'lemniscate'.")

        print("Landing the drone...")
        self.drone.land()
        self.display_battery_status()


def main():
    """
    Main function to run the drone trajectory experiment.
    """
    drone = Bebop2(drone_type="gazebo", ip_address="192.168.0.202")
    controller = DroneController(drone, trajectory_type="ellipse")
    controller.run_experiment()


if __name__ == "__main__":
    main()


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
