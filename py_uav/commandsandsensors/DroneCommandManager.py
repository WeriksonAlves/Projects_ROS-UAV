"""

"""

from .DroneSensorManager import DroneSensorManager
from ..ros.DroneCamera import DroneCamera
from ..ros.DroneControl import DroneControl
import numpy as np
import os
import rospy


class DroneCommandManager:
    """
    Manages commands issued to the drone, including validation, state
    management, and camera controls.
    """

    def __init__(self, drone_type: str, frequency: int,
                 sensor_manager: DroneSensorManager, main_dir: str) -> None:
        """
        Initializes the DroneCommandManager.

        :param drone_type: Type of the drone.
        :param frequency: Frequency of the drone's control loop.
        :param sensor_manager: Sensor manager for the drone.
        """
        self.drone_camera = DroneCamera(drone_type, main_dir, frequency)
        self.drone_control = DroneControl(drone_type, frequency)
        self.sensor_manager = sensor_manager
        self.frequency = frequency
        self.main_dir = main_dir
        self.snapshot_counter = 0

    # Drone Command Methods

    def _validate_command(self, is_emergency_allowed: bool = False,
                          is_hovering_required: bool = False,
                          is_landed_required: bool = False) -> bool:
        """
        Validates whether a command can be executed based on drone state.

        :param is_emergency_allowed: Whether the command is allowed during
                                        emergency state.
        :param is_hovering_required: Whether the drone must be hovering to
                                        xecute the command.
        :param is_landed_required: Whether the drone must be landed to execute
                                    the command.
        :return: True if the command is valid, False otherwise.
        """
        if self.sensor_manager.is_emergency() and not is_emergency_allowed:
            rospy.logwarn("Command ignored: Emergency state active.")
            return False
        if is_hovering_required and not self.sensor_manager.is_hovering():
            rospy.logwarn("Command ignored: Drone is not hovering.")
            return False
        if is_landed_required and not self.sensor_manager.is_landed():
            rospy.logwarn("Command ignored: Drone is not landed.")
            return False
        return True

    def takeoff(self) -> None:
        """Commands the drone to take off."""
        if not self._validate_command(is_landed_required=True):
            return
        rospy.loginfo("Initiating takeoff...")
        self.drone_control.takeoff()
        self.sensor_manager.change_status_flags('landed', False)
        self.sensor_manager.change_status_flags('hovering', True)
        rospy.loginfo("Drone has taken off.")

    def land(self) -> None:
        """Commands the drone to land."""
        if not self._validate_command(is_hovering_required=True):
            return
        rospy.loginfo("Initiating landing...")
        self.drone_control.land()
        self.sensor_manager.change_status_flags('hovering', False)
        self.sensor_manager.change_status_flags('landed', True)
        rospy.loginfo("Drone has landed.")

    def safe_takeoff(self, heigth: float = 0.5, timeout: float = 3.0) -> bool:
        """
        Safely takes off the drone within a given timeout.

        :param heigth: heigth to take off
        :param timeout: Maximum duration for the takeoff operation (seconds).
        :return: True if the drone successfully takes off, False otherwise.
        """
        if not self._validate_command(is_landed_required=True):
            return False

        rospy.loginfo("Starting safe takeoff...")
        start_time = rospy.get_time()

        while rospy.get_time() - start_time < timeout:
            self.drone_control.takeoff()
            rospy.sleep(0.1)
            if self.sensor_manager.get_sensor_data().get('altitude', 0.0
                                                         ) > heigth:
                self.sensor_manager.change_status_flags('landed', False)
                self.sensor_manager.change_status_flags('hovering', True)
                rospy.loginfo("Safe takeoff complete.")
                return True

        rospy.logwarn("Safe takeoff failed: Timeout exceeded.")
        return False

    def safe_land(self, heigth: float = 0.15, timeout: float = 3.0) -> None:
        """
        Safely lands the drone within a given timeout.

        :param heigth: heigth to land
        :param timeout: Maximum duration for the landing operation (seconds).
        """
        if not self._validate_command(is_hovering_required=True):
            return

        rospy.loginfo("Starting safe landing...")
        start_time = rospy.get_time()

        while rospy.get_time() - start_time < timeout:
            self.drone_control.land()
            rospy.sleep(0.1)
            if self.sensor_manager.get_sensor_data().get('altitude', 1.0
                                                         ) < heigth:
                self.sensor_manager.change_status_flags('hovering', False)
                self.sensor_manager.change_status_flags('landed', True)
                rospy.loginfo("Safe landing complete.")
                return

        rospy.logwarn("Safe landing failed: Timeout exceeded.")

    def emergency_stop(self, heigth: float = 0.15) -> None:
        """Executes an immediate emergency stop."""
        rospy.loginfo("Executing emergency stop...")
        while self.sensor_manager.get_sensor_data().get('altitude', 1.0
                                                        ) > heigth:
            self.drone_control.land()
            rospy.sleep(0.1)
        self.sensor_manager.change_status_flags('hovering', False)
        self.sensor_manager.change_status_flags('landed', True)
        self.sensor_manager.change_status_flags('emergency', True)
        rospy.loginfo("Emergency stop completed.")

    def flip(self, direction: str) -> None:
        """
        Commands the drone to perform a flip in the specified direction.

        :param direction: Direction of the flip (e.g., 'left', 'right').
        """
        valid_directions = {'left', 'right', 'forward', 'backward'}
        if direction not in valid_directions:
            rospy.logwarn(f"Invalid flip direction: {direction}.")
            return
        if not self._validate_command(is_hovering_required=True):
            return

        rospy.loginfo(f"Executing flip command: {direction}.")
        self.drone_control.flip(direction)
        rospy.loginfo(f"Flip {direction} completed.")

    def fly_direct(self, linear_x: float = 0.0, linear_y: float = 0.0,
                   linear_z: float = 0.0, angular_z: float = 0.0,
                   duration: float = 0.0) -> None:
        """
        Commands the drone to move directly with specified velocities.

        :param linear_x: Linear velocity along x-axis.
        :param linear_y: Linear velocity along y-axis.
        :param linear_z: Linear velocity along z-axis.
        :param angular_z: Angular velocity along z-axis.
        :param duration: Duration of movement (0 for indefinite movement).
        """
        if not self._validate_command(is_hovering_required=True):
            return

        rospy.loginfo("Executing direct flight command...")
        if duration > 0:
            start_time = rospy.get_time()
            while duration == 0 or rospy.get_time() - start_time < duration:
                self.drone_control.move(linear_x, linear_y, linear_z,
                                        angular_z)
                rospy.sleep(0.1)
            self.drone_control.move(0.0, 0.0, 0.0, 0.0)
        else:
            self.drone_control.move(linear_x, linear_y, linear_z, angular_z)
        rospy.loginfo("Direct flight command completed.")

    def move_relative(self, delta_x: float = 0.0, delta_y: float = 0.0,
                      delta_z: float = 0.0, delta_yaw: float = 0.0,
                      power: int = 0.25, theshold: float = 0.1) -> None:
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

            if error < theshold:
                break

            self.drone_control.move(error_vector['x'],
                                    -error_vector['y'],
                                    -error_vector['z'],
                                    error_vector['yaw'])
            rospy.sleep(1 / self.frequency)

        self.drone_control.move(0.0, 0.0, 0.0, 0.0)
        rospy.loginfo("The drone has stopped moving!")

    # Camera Control Methods

    def adjust_camera_orientation(self, tilt: float, pan: float) -> None:
        """
        Adjusts the drone camera's orientation.

        :param tilt: Tilt angle (degrees).
        :param pan: Pan angle (degrees).
        """
        rospy.loginfo(f"Adjusting camera orientation: Tilt={tilt}, Pan={pan}.")
        self.drone_camera.control_camera_orientation(tilt, pan)

    def adjust_camera_exposure(self, exposure: float) -> None:
        """
        Adjusts the camera exposure setting.

        :param exposure: Exposure value (-3 to 3).
        """
        rospy.loginfo(f"Adjusting camera exposure: {exposure}.")
        self.drone_camera.adjust_exposure(exposure)

    def release_camera(self) -> None:
        """Releases camera resources."""
        rospy.loginfo("Releasing camera resources.")
        self.drone_camera.release()

    def save_snapshot(self, frame: np.ndarray) -> None:
        """
        Save a snapshot from the drone's camera to the specified directory.

        :param frame: The image frame to save.
        """
        filename = self._generate_unique_snapshot_filename()
        self.drone_camera.capture_snapshot(frame, filename)

    def _generate_unique_snapshot_filename(self) -> str:
        """
        Generate a unique filename for saving snapshots.

        :return: A unique filename.
        """
        snapshot_dir = os.path.join(self.main_dir, 'images', 'snapshot')
        os.makedirs(snapshot_dir, exist_ok=True)

        filename = f"img_{self.snapshot_counter:04d}.png"
        self.snapshot_counter += 1
        return os.path.join(snapshot_dir, filename)
