import os
import numpy as np
import rospy
from .commandsandsensors.DroneCommandManager import DroneCommandManager
from .commandsandsensors.DroneSensorManager import DroneSensorManager
from .ros.DroneCamera import DroneCamera
from .ros.DroneControl import DroneControl
from .ros.DroneManagers import GPSStateManager, HealthMonitor, ParameterManager
from .ros.DroneMedia import DroneMedia
from .ros.DroneSensors import DroneSensors
from .ros.DroneStates import FlightStateManager
from typing import Callable, Optional, Tuple


class Bebop2:
    """
    A singleton class that manages communication with the Bebop drone,
    handling video capture, sensor updates, and camera control via ROS.
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(Bebop2, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str = "Bebop2",
                 ip_address: str = "192.168.0.202", frequency: int = 30):
        """
        Initializes the Bebop2 instance with specified drone type and IP
        address.

        :param drone_type: Type of drone, default is 'Bebop2'.
        :param ip_address: The drone's IP address, default is '192.168.0.202'.
        :param frequency: Command frequency (Hz).
        """
        self.drone_type = drone_type
        self.ip_address = ip_address
        self.frequency = frequency
        self.command_interval = 1.0 / frequency
        self.last_command_time = rospy.get_time()
        self.state_map = self._initialize_state_map()
        self.main_dir = os.path.dirname(__file__)

        self._initialize_components()

        if not self._check_connection():
            rospy.logerr("Bebop2 not connected to the network.")
            exit(1)

    def _initialize_components(self) -> None:
        """Initializes all ROS-drone components."""
        self.camera = DroneCamera(self.drone_type, self.frequency)
        self.control = DroneControl(self.drone_type, self.frequency)
        self.gps = GPSStateManager(self.drone_type, self.frequency)
        self.health = HealthMonitor(self.drone_type, self.frequency)
        self.params = ParameterManager(self.drone_type, self.frequency)
        self.media = DroneMedia(self.drone_type, self.frequency)
        self.sensors = DroneSensors(self.drone_type, self.frequency)
        self.states = FlightStateManager(self.drone_type, self.frequency)

        self.command_manager = DroneCommandManager(self.control, self.sensors,
                                                   self.state_map)
        self.sensor_manager = DroneSensorManager(self.sensors)

    @staticmethod
    def _initialize_state_map() -> dict:
        """Defines state mapping for the drone."""
        return {
            'E': 'emergency',
            'H': 'hovering',
            'L': 'landed',
            'l': 'landing',
            'M': 'moving',
            'T': 'takingoff'
        }

    def _is_time_to_command(self) -> bool:
        """
        Verifies if enough time has passed since the last command.

        :return: True if the command interval has passed; False otherwise.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    def _check_connection(self) -> bool:
        """
        Checks if the drone is connected to the network.

        :return: True if connected, False otherwise.
        """
        return os.system(f"ping -c 1 {self.ip_address}") == 0

    def set_user_sensor_callback(self, callback: Callable, args: tuple
                                 ) -> None:
        """
        Sets the user-defined callback function for sensor updates.

        :param callback: Function to call upon sensor update.
        :param args: Arguments to pass to the callback function.
        """
        self.sensors.set_user_callback_function(callback, args)

    def update_sensors(self, raw_data) -> None:
        self.sensor_manager.update_sensors(raw_data)

    # Drone Control Methods

    def _is_state(self, state_key: str) -> bool:
        """Checks if the drone is in a specific state."""
        return self.sensors.flying_state == self.state_map.get(state_key)

    def takeoff(self) -> None:
        """Initiates drone takeoff if in landed state and not in emergency."""
        if self._is_state('E'):
            rospy.loginfo("Bebop is in emergency mode.")
            return
        if self._is_state('L'):
            self.control.takeoff()
            self.sensors.flying_state = self.state_map['H']
            rospy.loginfo("Bebop is taking off.")
        else:
            rospy.loginfo("Bebop is already airborne or busy.")

    def safe_takeoff(self, timeout: int = 5) -> None:
        """
        Safely takes off, ensuring the drone reaches hovering within timeout.

        :param timeout: Max time to achieve hover (seconds).
        """
        if self._is_state('E') or not self._is_state('L'):
            rospy.loginfo("Takeoff not allowed in current state.")
            return

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < timeout:
            self.control.takeoff()
            rospy.sleep(0.1)
            if self.sensors.get_altitude() > 0.5:
                self.sensors.flying_state = self.state_map['H']
                rospy.loginfo("Bebop is hovering.")
                return
        rospy.logwarn("Safe takeoff timeout reached.")

    def land(self) -> None:
        """Initiates landing if in hovering state and not in emergency."""
        if self._is_state('E'):
            rospy.loginfo("Bebop is in emergency mode.")
            return
        if self._is_state('H'):
            self.control.land()
            self.sensors.flying_state = self.state_map['L']
            rospy.loginfo("Bebop is landing.")
        else:
            rospy.loginfo("Bebop is not hovering.")

    def safe_land(self, timeout: int = 5) -> None:
        """
        Safely lands, ensuring the drone lands within the timeout.

        :param timeout: Maximum time to wait for the drone to land (seconds).
        """
        if self._is_state('E'):
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if self._is_state('H'):
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < timeout:
                self.control.land()
                rospy.sleep(0.1)
                if self.sensors.get_altitude() < 0.15:
                    self.sensors.flying_state = self.state_map['L']
                    rospy.loginfo("Bebop has landed.")
                    return
        else:
            rospy.loginfo("Bebop is already landed or busy.")

    def smart_sleep(self, seconds: float) -> None:
        """
        Sleeps for the specified duration but wakes for all Wi-Fi
        notifications.

        :param seconds: Duration to sleep in seconds.
        """
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < seconds:
            rospy.sleep(0.1)

    def _normalize_command(self, value: float) -> float:
        """Clamps command value between -1 and 1."""
        return max(min(value / 100.0, 1.0), -1.0)

    def fly_direct(self, roll: float, pitch: float, yaw: float,
                   vertical: float, duration: float) -> None:
        """
        Sends control signals to the drone for direct flight.

        :param roll: Roll value (-100 to 100).
        :param pitch: Pitch value (-100 to 100).
        :param yaw: Yaw value (-100 to 100).
        :param vertical: Vertical speed value (-100 to 100).
        :param duration: Command duration (seconds).
        """
        if self._is_state('E') or not self._is_state('H'):
            rospy.logwarn("Direct flight not allowed in current state.")
            return

        roll, pitch, yaw, vertical = map(self._normalize_command,
                                         [roll, pitch, yaw, vertical])
        start_time = rospy.get_time()

        while rospy.get_time() - start_time < duration:
            self.control.move(roll, pitch, vertical, yaw)
            rospy.sleep(0.1)
        self.control.move(0, 0, 0, 0)

    def flip(self, direction: str) -> None:
        """
        Performs a flip in the specified direction.

        :param direction: Direction for the flip.
        """
        if self._is_state('E'):
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if not self._is_state('H'):
            rospy.logwarn("Bebop must be hovering to flip.")
            return

        if direction.lower() not in ['forward', 'backward', 'left', 'right']:
            rospy.logwarn(f"Invalid flip direction: {direction}. Valid options"
                          f" are ['forward', 'backward', 'left', 'right'].")
            return

        self.control.flip(direction)
        rospy.loginfo(f"Flip command executed in {direction} direction.")

    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                      dr: float = 0.0, velocity: float = 0.1) -> None:
        """
        Moves the drone relative to its current position based on distances
        provided.

        :param dx: Distance to move along the X-axis (in meters).
        :param dy: Distance to move along the Y-axis (in meters).
        :param dz: Distance to move along the Z-axis (in meters).
        :param dr: Rotation in radians around the Z-axis.
        :param velocity: The movement velocity.
        """
        if self._is_state('E'):
            rospy.loginfo("Drone is in emergency mode.")
            return

        desired_position = np.array([dx, dy, dz, dr])
        current_position = np.array(self.sensors.sensors_dict['X'])[[0, 1, 2, 5]]  # [X, Y, Z, Rotation]
        error = desired_position - current_position

        # Loop until error in any dimension is reduced below a threshold
        while (np.abs(error) >= 0.1).any():
            movement_vector = np.sign(error) * velocity
            self.control.move(*movement_vector)
            error = desired_position - np.array(self.sensors.sensors_dict['X'])[[0, 1, 2, 5]]  # Recalculate error

        # Stop movement after reaching the target
        self.control.move(0.0, 0.0, 0.0, 0.0)
        rospy.loginfo(f"Moved drone by dx={dx}, dy={dy}, dz={dz}, dr={dr}.")
        rospy.loginfo("Drone has reached the desired position.")

    # Camera Control Methods

    def capture_frame(self, subscriber: str = 'compressed') -> Tuple[
            bool, Optional[np.ndarray]]:
        """
        Captures the latest camera frame.

        :param subscriber: Frame type.
        :return: Tuple with success flag and image data.
        """
        flag = self.camera.success_flags.get(subscriber, False)
        data = self.camera.image_data.get(subscriber)
        return flag, data

    def release_camera(self) -> None:
        """Releases camera resources and resets internal state."""
        self.camera.reset_state()
        self.camera.success_flags.clear()
        self.camera.image_data.clear()
        rospy.loginfo("Camera resources released.")

    def pan_tilt_camera(self, tilt_degrees: float, pan_degrees: float,
                        pitch: float = 0.0, yaw: float = 0.0) -> None:
        """
        Adjusts the camera's tilt and pan.

        :param tilt_degrees: Vertical camera movement.
        :param pan_degrees: Horizontal camera movement.
        :param pitch: Compensation for drone pitch.
        :param yaw: Compensation for drone yaw.
        """
        self.camera.control_camera_orientation(tilt_degrees - pitch,
                                               pan_degrees - yaw)

    def take_snapshot(self, frame: np.ndarray) -> None:
        """
        Saves the current frame as a snapshot.

        :param frame: Image frame to save.
        """
        filename = self._get_unique_snapshot_filename()
        self.camera._save_image(frame, filename)

    def _get_unique_snapshot_filename(self) -> str:
        """Generates a unique filename for the snapshot."""
        base_filename = os.path.join(self.main_dir, 'images', 'snapshot_')
        i = 0
        while True:
            filename = f"{base_filename}{str(i).zfill(4)}.png"
            if not os.path.exists(filename):
                return filename
            i += 1

    def set_exposure(self, exposure_value: float) -> None:
        """
        Adjusts the camera's exposure setting.

        :param exposure_value: Exposure value to set (-3.0 to 3.0).
        """
        self.camera.adjust_exposure(exposure_value)
