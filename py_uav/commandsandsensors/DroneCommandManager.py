import numpy as np
import rospy
from .DroneSensorManager import DroneSensorManager
from ..ros.DroneCamera import DroneCamera
from ..ros.DroneControl import DroneControl


class DroneCommandManager:
    """
    Manages commands issued to the drone, including validation and state
    management.
    """

    def __init__(self, drone_type: str, frequency: int,
                 sensor_manager: DroneSensorManager) -> None:
        """
        Initializes the DroneCommandManager.

        :param drone_type: Type of the drone.
        :param frequency: Frequency of the drone's control loop.
        :param sensor_manager: Sensor manager for the drone.
        """
        self.drone_camera = DroneCamera(drone_type, frequency)
        self.drone_control = DroneControl(drone_type, frequency)
        self.sensor_manager = sensor_manager

    # Drone commands

    def takeoff(self) -> None:
        """Commands the drone to take off, with validation."""
        if self.sensor_manager.is_emergency():
            rospy.loginfo("Cannot take off: Emergency state.")
            return
        if not self.sensor_manager.is_landed():
            rospy.loginfo("Takeoff command ignored. Drone not landed.")
            return
        rospy.loginfo("Initiating takeoff...")
        self.sensor_manager.change_status_flags('landed', False)
        self.drone_control.takeoff()
        self.sensor_manager.change_status_flags('hovering', True)
        rospy.loginfo("Drone taking off.")

    def land(self) -> None:
        """Commands the drone to land, with validation."""
        if self.sensor_manager.is_emergency():
            rospy.loginfo("Cannot land: Emergency state.")
            return
        if not self.sensor_manager.is_hovering():
            rospy.loginfo("Land command ignored. Drone not hovering.")
            return
        rospy.loginfo("Initiating landing...")
        self.sensor_manager.change_status_flags('hovering', False)
        self.drone_control.land()
        self.sensor_manager.change_status_flags('landed', True)
        rospy.loginfo("Drone landing.")

    def safe_takeoff(self, timeout: float = 3.0) -> bool:
        """
        Safely takes off the drone, with a timeout.

        :param timeout: Maximum time to attempt takeoff.
        """
        if self.sensor_manager.is_emergency():
            rospy.loginfo("Cannot take off: Emergency state.")
            return
        if not self.sensor_manager.is_landed():
            rospy.loginfo("Safe takeoff ignored. Drone not landed.")
            return

        start_time = rospy.get_time()
        while (not self.sensor_manager.is_hovering()) and (
                rospy.get_time() - start_time < timeout):
            rospy.loginfo("Attempting safe takeoff.")
            self.drone_control.takeoff()
            rospy.sleep(0.1)
            if self.sensor_manager.get_sensor_data()['altitude'] > 0.5:
                self.sensor_manager.change_status_flags('landed', False)
                rospy.loginfo("Drone safely took off.")
                self.sensor_manager.change_status_flags('hovering', True)
                return True
        rospy.loginfo("Drone failed to take off.")
        return False

    def safe_land(self, timeout: float = 3.0) -> None:
        """
        Safely lands the drone, with a timeout.

        :param timeout: Maximum time to attempt landing.
        """
        if self.sensor_manager.is_emergency():
            rospy.loginfo("Cannot land: Emergency state.")
            return
        if not self.sensor_manager.is_hovering():
            rospy.loginfo("Safe land ignored. Drone not hovering.")
            return

        start_time = rospy.get_time()
        while (not self.sensor_manager.is_landed()) and (
                rospy.get_time() - start_time < timeout):
            rospy.loginfo("Attempting safe landing.")
            self.drone_control.land()
            rospy.sleep(0.1)
            if self.sensor_manager.get_sensor_data()['altitude'] < 0.15:
                self.sensor_manager.change_status_flags('hovering', False)
                rospy.loginfo("Drone safely landed.")
                self.sensor_manager.change_status_flags('landed', True)
                return
        rospy.loginfo("Drone failed to land.")

    def emergency_stop(self) -> None:
        """Stops the drone immediately in an emergency situation."""
        if self.sensor_manager.is_emergency():
            rospy.loginfo("Drone already in emergency mode.")
            return

        rospy.loginfo("Attempting emergency stop...")
        while self.sensor_manager.get_sensor_data()['altitude'] > 0.15:
            self.drone_control.land()
            rospy.sleep(0.1)
        self.drone_control.land()
        self.sensor_manager.change_status_flags('hovering', False)
        self.sensor_manager.change_status_flags('landed', True)
        self.sensor_manager.change_status_flags('emergency', True)
        rospy.loginfo("Drone emergency stopped.")

    def flip(self, direction: str) -> None:
        """
        Flips the drone in the specified direction.

        :param direction: Direction of flip.
        """
        if direction not in {'left', 'right', 'forward', 'backward'}:
            rospy.loginfo(f"Invalid flip direction: {direction}.")
            return
        if not self.sensor_manager.is_hovering():
            rospy.loginfo("Flip command ignored. Drone not hovering.")
            return
        self.drone_control.flip(direction)
        rospy.loginfo(f"Drone flipping {direction}.")

    def fly_direct(self, linear_x: float = 0.0, linear_y: float = 0.0,
                   linear_z: float = 0.0, angular_z: float = 0.0,
                   duration: float = 0.0) -> None:
        """
        Commands the drone to move directly in the specified direction.

        :param linear_x: Linear velocity in the x-axis.
        :param linear_y: Linear velocity in the y-axis.
        :param linear_z: Linear velocity in the z-axis.
        :param angular_z: Angular velocity in the z-axis.
        :param duration: Duration of movement.
        """
        if self.sensor_manager.is_emergency():
            rospy.loginfo("Cannot take off: Emergency state.")
            return
        if not self.sensor_manager.is_hovering():
            rospy.loginfo("Cannot move: Drone not hovering.")
            return

        rospy.loginfo("Drone moving in specified direction.")
        if duration == 0.0:
            self.drone_control.move(linear_x, linear_y, linear_z, angular_z)
        else:
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < duration:
                self.drone_control.move(linear_x, linear_y, linear_z,
                                        angular_z)
                rospy.sleep(0.1)
            self.drone_control.move(0.0, 0.0, 0.0, 0.0)
            rospy.loginfo("Movement complete.")

    def move_relative(self, delta_x: float = 0.0, delta_y: float = 0.0,
                      delta_z: float = 0.0, delta_yaw: float = 0.0,
                      power: int = 0.25) -> None:
        """
        Moves the drone in the specified relative direction.

        :param delta_x: Change in x-axis.
        :param delta_y: Change in y-axis.
        :param delta_z: Change in z-axis.
        :param delta_yaw: Change in yaw.
        :param power: Power of the movement [0 to 1].
        """
        if self.sensor_manager.is_emergency():
            rospy.loginfo("Cannot move: Emergency mode!")
            return
        if not self.sensor_manager.is_hovering():
            rospy.loginfo("Cannot move: Drone not hovering.")
            return

        target_position = {
            'x': self.sensor_manager.get_sensor_data()[
                'position'][0] + delta_x,
            'y': self.sensor_manager.get_sensor_data()[
                'position'][1] + delta_y,
            'z': self.sensor_manager.get_sensor_data()[
                'position'][2] + delta_z,
            'yaw': self.sensor_manager.get_sensor_data()[
                'orientation'][2] + delta_yaw
        }

        rospy.loginfo("Moving the drone to a relative position.")
        while True:
            current_position = {
                'x': self.sensor_manager.get_sensor_data()['position'][0],
                'y': self.sensor_manager.get_sensor_data()['position'][1],
                'z': self.sensor_manager.get_sensor_data()['position'][2],
                'yaw': self.sensor_manager.get_sensor_data()['orientation'][2]
            }

            error_vector = {k: np.tanh(target_position[k] - current_position[k]
                                       ) * (power) for k in current_position}
            error = np.linalg.norm(list(error_vector.values()))

            if error < 0.1:
                break

            self.drone_control.move(error_vector['x'],
                                    -error_vector['y'],
                                    -error_vector['z'],
                                    error_vector['yaw'])
            rospy.sleep(1 / 30)

        self.drone_control.move(0.0, 0.0, 0.0, 0.0)
        rospy.loginfo("The drone has stopped moving!")

    # Camera commands

    def adjust_camera_orientation(self, tilt: float, pan: float) -> None:
        """
        Adjusts the camera orientation by setting its tilt and pan.

        :param tilt: Vertical movement in degrees.
        :param pan: Horizontal movement in degrees.
        """
        self.drone_camera.control_camera_orientation(tilt, pan)

    def adjust_camera_exposure(self, exposure: float) -> None:
        """
        Adjusts the camera's exposure setting.

        :param exposure: Exposure value to set for the camera [-3 to 3].
        """
        self.drone_camera.adjust_exposure(exposure)

    def release_camera(self) -> None:
        """Releases the camera resources used by the drone."""
        self.drone_camera.release()
