import os
import numpy as np
import rospy
import threading
from typing import Callable, Dict, Tuple

from .interfaces.RosCommunication import RosCommunication
from .ros.DroneCamera import DroneCamera
from .ros.DroneControl import DroneControl
from .ros.DroneManagers import GPSStateManager, HealthMonitor, ParameterManager
from .ros.DroneMedia import DroneMedia
from .ros.DroneSensors import DroneSensors
from .ros.DroneStates import FlightStateManager


class Bebop2ROS:
    """
    Manages interaction between a Bebop2 drone and ROS, providing control and
    sensor access.
    """

    def __init__(self, drone_type: str = 'bebop2',
                 ip_address: str = "192.168.0.202", frequency: float = 30.0
                 ) -> None:
        """
        Initializes Bebop2ROS with drone type, IP address, and sensor update
        frequency.

        :param drone_type: Type of the drone.
        :param ip_address: IP address for connectivity checks.
        :param frequency: Frequency of sensor updates in Hz.
        """
        self.drone_type = drone_type
        self.ip_address = ip_address
        self.frequency = frequency

        # Initialize subsystems
        self.sensors = DroneSensors(self.drone_type, self.frequency)
        self.ros_communication = RosCommunication()
        self.drone_control = DroneControl()
        self.drone_camera = DroneCamera()
        self.gps_state_manager = GPSStateManager()
        self.health_monitor = HealthMonitor()
        self.parameter_manager = ParameterManager()
        self.drone_media = DroneMedia()
        self.flight_state_manager = FlightStateManager()

        # Initialize sensor data and state flags
        self.sensors_data = self._initialize_sensor_data()
        self.status_flags = self._initialize_status_flags()

        self.user_callback: Callable = None
        self.user_callback_args = None

        # Start sensor update thread
        self.update_thread = threading.Thread(target=self._update_sensors)
        self.update_thread.daemon = True
        self.update_thread.start()

    def _initialize_sensor_data(self) -> Dict[str, object]:
        """Initializes sensor data dictionary with default values."""
        return {
            'altitude': 0.0, 'attitude': [0.0] * 3, 'battery': 100,
            'camera': None, 'gps': [0.0] * 3, 'image': None,
            'orientation': [0.0] * 3, 'position': [0.0] * 3,
            'speed_angular': [0.0] * 3, 'speed_linear': [0.0] * 3,
            'state': "unknown", 'wifi_signal': 0.0
        }

    def _initialize_status_flags(self) -> Dict[str, bool]:
        """Initializes status flags for the drone."""
        return {
            'connected': False, 'battery_full': False, 'battery_low': False,
            'battery_critical': False, 'emergency': False, 'hovering': False,
            'landed': True, 'moving': False, 'image_on': False,
            'recording': False, 'video_on': False, 'stabilized': False,
            'automatic': False, 'manual': False, 'gps_fixed': False,
            'gps_updated': False, 'temperature_updated': False,
            'pressure_updated': False, 'state_updated': False
        }

    def _update_sensors(self) -> None:
        """Updates sensor data at a specified frequency."""
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            new_data = self.sensors.get_processed_sensor_data()
            for key, value in new_data.items():
                self.sensors_data[key] = value
            rate.sleep()

    def check_connection(self) -> None:
        """Checks if the drone is connected to the network."""
        self.status_flags['connected'] = os.system(
            f"ping -c 1 {self.ip_address}") == 0

    def smart_sleep(self, seconds: float) -> None:
        """
        Sleeps with periodic ROS check.

        :param seconds: Duration of sleep in seconds.
        """
        rospy.sleep(max(0.1, seconds))

    def set_user_callback(self, callback: Callable, *args) -> None:
        """
        Registers a user-defined callback function to handle custom events.

        :param callback: Callable function to register as a callback.
        :param args: Additional arguments to pass to the callback.
        """
        self.user_callback = callback
        self.user_callback_args = args

    def get_sensor_data(self) -> Dict[str, object]:
        """Returns the latest sensor data."""
        return self.sensors_data

    def trigger_callback(self) -> None:
        """Executes the user-defined callback with provided arguments."""
        if self.user_callback:
            self.user_callback(*self.user_callback_args)

    # Public Drone Control Methods

    def land(self) -> None:
        """Commands the drone to land."""
        if self.is_emergency:
            rospy.loginfo("Cannot land: Emergency state.")
            return
        if not self.is_hovering:
            rospy.loginfo("Land command ignored. Drone not hovering.")
            return
        self.drone_control.land()
        self.is_hovering, self.is_landed = False, True
        rospy.loginfo("Drone landing.")

    def takeoff(self) -> None:
        """Commands the drone to take off."""
        if self.is_emergency:
            rospy.loginfo("Cannot take off: Emergency state.")
            return
        if not self.is_landed:
            rospy.loginfo("Takeoff command ignored. Drone not landed.")
            return
        self.drone_control.takeoff()
        self.is_hovering, self.is_landed = True, False
        rospy.loginfo("Drone taking off.")

    def safe_takeoff(self, timeout: float = 3.0) -> None:
        """
        Safely takes off the drone.

        :param timeout: Maximum time to attempt takeoff.
        """
        if self.is_emergency:
            rospy.loginfo("Cannot take off: Emergency state.")
            return
        if not self.is_landed:
            rospy.loginfo("Safe takeoff ignored. Drone not landed.")
            return

        start_time = rospy.get_time()
        while not self.is_hovering and rospy.get_time() - start_time < timeout:
            rospy.loginfo("Attempting safe takeoff.")
            self.drone_control.takeoff()
            rospy.sleep(0.1)
            if self.sensors_data['altitude'] > 0.5:
                self.is_hovering, self.is_landed = True, False
                rospy.loginfo("Drone safely took off.")
                return
        rospy.loginfo("Drone failed to take off.")

    def safe_land(self, timeout: float = 3.0) -> None:
        """
        Safely lands the drone.

        :param timeout: Maximum time to attempt landing.
        """
        if self.is_emergency:
            rospy.loginfo("Cannot land: Emergency state.")
            return
        if not self.is_hovering:
            rospy.loginfo("Safe land ignored. Drone not hovering.")
            return

        start_time = rospy.get_time()
        while not self.is_landed and rospy.get_time() - start_time < timeout:
            rospy.loginfo("Attempting safe landing.")
            self.drone_control.land()
            rospy.sleep(0.1)
            if self.sensors_data['altitude'] < 0.15:
                self.is_hovering, self.is_landed = False, True
                rospy.loginfo("Drone safely landed.")
                return
        rospy.loginfo("Drone failed to land.")

    def emergency_stop(self) -> None:
        """Stops the drone immediately."""
        while self.sensors_data['altitude'] > 0.15:
            rospy.loginfo("Attempting emergency stop.")
            self.drone_control.land()
            rospy.sleep(0.1)
        self.is_emergency = True
        self.is_hovering, self.is_landed = False, True
        rospy.loginfo("Drone emergency stopped.")

    def flip(self, direction: str) -> None:
        """
        Flips the drone in the specified direction.

        :param direction: Direction of flip.
        """
        if direction in {'left', 'right', 'forward', 'backward'
                         } and self.is_hovering:
            self.drone_control.flip(direction)
            rospy.loginfo(f"Drone flipping {direction}.")
        else:
            rospy.loginfo("Flip command ignored: Invalid direction or drone"
                          " not hovering.")

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
        if not self.is_hovering:
            rospy.loginfo("Cannot move: Drone not hovering.")
            return
        rospy.loginfo("Drone moving in specified direction.")
        if duration == 0.0:
            self.drone_control.move(
                self._normalize_command(linear_x),
                self._normalize_command(linear_y),
                self._normalize_command(linear_z),
                self._normalize_command(angular_z)
            )
        else:
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < duration:
                self.drone_control.move(
                    self._normalize_command(linear_x),
                    self._normalize_command(linear_y),
                    self._normalize_command(linear_z),
                    self._normalize_command(angular_z)
                )
                rospy.sleep(0.1)
            self.drone_control.move(0.0, 0.0, 0.0, 0.0)
            rospy.loginfo("Movement complete.")

    def _normalize_command(self, value: float) -> float:
        """Normalizes command values to the range [-1, 1]."""
        return max(min(value / 100.0, 1.0), -1.0)

    def move_relative(self, delta_x: float = 0.0, delta_y: float = 0.0,
                      delta_z: float = 0.0, delta_yaw: float = 0.0,
                      power: int = 20) -> None:
        """
        Moves the drone in the specified relative direction.

        :param delta_x: Change in x-axis.
        :param delta_y: Change in y-axis.
        :param delta_z: Change in z-axis.
        :param delta_yaw: Change in yaw.
        :param power: Power of the movement [0 to 100].
        """
        if self.is_emergency:
            rospy.loginfo("Cannot move: Emergency mode!")
            return
        if not self.is_hovering:
            rospy.loginfo("Cannot move: Drone not hovering.")
            return

        target_position = {
            'x': self.sensors_data['position'][0] + delta_x,
            'y': self.sensors_data['position'][1] + delta_y,
            'z': self.sensors_data['position'][2] + delta_z,
            'yaw': self.sensors_data['orientation'][2] + delta_yaw
        }

        rospy.loginfo("Moving the drone to a relative position.")
        while True:
            current_position = {
                'x': self.sensors_data['position'][0],
                'y': self.sensors_data['position'][1],
                'z': self.sensors_data['position'][2],
                'yaw': self.sensors_data['orientation'][2]
            }
            error_vector = {k: np.tanh(target_position[k] - v) * (power / 100)
                            for k, v in current_position.items()}
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

    # Public Camera Methods

    def release_camera(self) -> None:
        """Releases the camera resources used by the drone."""
        self.drone_camera.release()

    def take_snapshot(self, frame: np.ndarray) -> None:
        """
        Captures a snapshot using the drone's camera and saves it to a unique
        file.

        :param frame: The image frame to be saved.
        """
        filename = self._generate_unique_snapshot_filename()
        self.drone_camera.save_image(frame, filename)

    def _generate_unique_snapshot_filename(self) -> str:
        """
        Generates a unique filename for saving snapshots.

        :return: A unique filename for the snapshot.
        """
        base_filename = os.path.join(self.main_dir, 'images', 'snapshot',
                                     'img_')
        i = 0
        while True:
            filename = f"{base_filename}{str(i).zfill(4)}.png"
            if not os.path.exists(filename):
                return filename
            i += 1

    def read_image(self, subscriber: str = 'compressed'
                   ) -> Tuple[bool, np.ndarray]:
        """
        Reads an image from the drone's camera.

        :param subscriber: The name of the image data subscriber to use.
        :return: A tuple containing a flag indicating success and the image
                 data.
        """
        flag = self.drone_camera.success_flags.get(subscriber, False)
        data = self.drone_camera.image_data.get(subscriber)
        if data is None or not flag:
            return False, np.array([])
        return flag, data

    def adjust_camera_orientation(self, tilt: float, pan: float,
                                  pitch_comp: float = 0.0,
                                  yaw_comp: float = 0.0) -> None:
        """
        Adjusts the camera orientation by setting its tilt and pan.

        :param tilt: Vertical movement in degrees.
        :param pan: Horizontal movement in degrees.
        :param pitch_comp: Optional pitch compensation for the drone.
        :param yaw_comp: Optional yaw compensation for the drone.
        """
        self.drone_camera.control_camera_orientation(tilt - pitch_comp,
                                                     pan - yaw_comp)

    def adjust_camera_exposure(self, exposure: float) -> None:
        """
        Adjusts the camera's exposure setting.

        :param exposure: Exposure value to set for the camera [-3 to 3].
        """
        self.drone_camera.adjust_exposure(exposure)
