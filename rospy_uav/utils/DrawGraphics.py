from mpl_toolkits.mplot3d import Axes3D
from typing import List, Optional, Tuple
import matplotlib.pyplot as plt
import numpy as np


class DrawGraphics:
    """
    A utility class for visualizing 3D trajectories and ensuring consistent
    plotting properties.
    """

    @staticmethod
    def set_axes_equal(ax: Axes3D) -> None:
        """
        Adjusts the scaling of a 3D plot so that all axes have equal range,
        maintaining proper aspect ratio.

        :param ax: The 3D axes object to adjust.
        """
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        # Calculate ranges and midpoints
        ranges = {
            "x": abs(x_limits[1] - x_limits[0]),
            "y": abs(y_limits[1] - y_limits[0]),
            "z": abs(z_limits[1] - z_limits[0]),
        }
        max_range = max(ranges.values())

        midpoints = {
            "x": np.mean(x_limits),
            "y": np.mean(y_limits),
            "z": np.mean(z_limits),
        }

        # Update axes limits
        for axis, midpoint in midpoints.items():
            ax.set_xlim3d(
                [midpoints["x"] - max_range / 2,
                 midpoints["x"] + max_range / 2])
            ax.set_ylim3d(
                [midpoints["y"] - max_range / 2,
                 midpoints["y"] + max_range / 2])
            ax.set_zlim3d(
                [midpoints["z"] - max_range / 2,
                 midpoints["z"] + max_range / 2])

    @staticmethod
    def create_3d_plot() -> Tuple[plt.Figure, Axes3D]:
        """
        Creates a 3D plot with a specified figure size.

        :return: A tuple containing the figure and axes objects.
        """
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection="3d")
        return fig, ax

    @staticmethod
    def plot_trajectory(trajectory_data: List[Tuple[float, float, float]],
                        fig: Optional[plt.Figure] = None, ax: Optional[
                            Axes3D] = None) -> None:
        """
        Visualizes the 3D trajectory of the drone based on the given data.

        :param trajectory_data: A list of (x, y, z) tuples representing the
                                drone's trajectory.
        :param fig: The figure object to use for plotting.
        :param ax: The 3D axes object to use for plotting.
        """
        if not trajectory_data:
            raise ValueError("Trajectory data cannot be empty.")

        # Extract coordinates
        x, y, z = zip(*trajectory_data)

        # Create 3D plot
        if not fig or not ax:
            fig, ax = DrawGraphics.create_3d_plot()

        # Plot trajectory
        ax.scatter(x, y, z, color="b", label="Trajectory", s=20)
        ax.plot(x, y, z, color="r", alpha=0.6)

        # Set labels and titles
        ax.set_title("Drone Trajectory", fontsize=16)
        ax.set_xlabel("X (meters)", fontsize=12)
        ax.set_ylabel("Y (meters)", fontsize=12)
        ax.set_zlabel("Z (meters)", fontsize=12)
        ax.legend(fontsize=10)

        # Apply equal axes scaling and grid
        DrawGraphics.set_axes_equal(ax)
        ax.grid(True)
