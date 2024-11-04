from .ros.DroneCamera import DroneCamera
from .ros.DroneControl import DroneControl
from .ros.DroneManagers import GPSStateManager, HealthMonitor, ParameterManager
from .ros.DroneMedia import DroneMedia
from .ros.DroneSensors import DroneSensors
from .ros.DroneStates import FlightStateManager
from typing import Callable, Optional, Tuple
import os
import rospy
import numpy as np


class BebopSensors:
    def __init__(self):
        self.sensors_dict = dict()
        self.RelativeMoveEnded = False
        self.CameraMoveEnded_tilt = False
        self.CameraMoveEnded_pan = False
        self.flying_state = "unknown"
        self.flat_trim_changed = False
        self.max_altitude_changed = False
        self.max_distance_changed = False
        self.no_fly_over_max_distance = False
        self.max_tilt_changed = False
        self.max_pitch_roll_rotation_speed_changed = False
        self.max_vertical_speed_changed = False
        self.max_rotation_speed = False
        self.hull_protection_changed = False
        self.outdoor_mode_changed = False
        self.picture_format_changed = False
        self.auto_white_balance_changed = False
        self.exposition_changed = False
        self.saturation_changed = False
        self.timelapse_changed = False
        self.video_stabilization_changed = False
        self.video_recording_changed = False
        self.video_framerate_changed = False
        self.video_resolutions_changed = False

        # default to full battery
        self.battery = 100

        # this is optionally set elsewhere
        self.user_callback_function = None

    def set_user_callback_function(self, function, args):
        """
        Sets the user callback function (called everytime the sensors are updated)

        :param function: name of the user callback function
        :param args: arguments (tuple) to the function
        :return:
        """
        self.user_callback_function = function
        self.user_callback_function_args = args

    def update(self, sensor_name, sensor_value, sensor_enum):
        if (sensor_name is None):
            print("Error empty sensor")
            return


        if (sensor_name, "enum") in sensor_enum:
            # grab the string value
            if (sensor_value is None or sensor_value > len(sensor_enum[(sensor_name, "enum")])):
                value = "UNKNOWN_ENUM_VALUE"
            else:
                enum_value = sensor_enum[(sensor_name, "enum")][sensor_value]
                value = enum_value

            self.sensors_dict[sensor_name] = value

        else:
            # regular sensor
            self.sensors_dict[sensor_name] = sensor_value

        # some sensors are saved outside the dictionary for internal use (they are also in the dictionary)
        if (sensor_name == "FlyingStateChanged_state"):
            self.flying_state = self.sensors_dict["FlyingStateChanged_state"]

        if (sensor_name == "PilotingState_FlatTrimChanged"):
            self.flat_trim_changed = True

        if (sensor_name == "moveByEnd_dX"):
            self.RelativeMoveEnded = True

        if (sensor_name == "OrientationV2_tilt"):
            self.CameraMoveEnded_tilt = True

        if (sensor_name == "OrientationV2_pan"):
            self.CameraMoveEnded_pan = True

        if (sensor_name == "MaxAltitudeChanged_current"):
            self.max_altitude_changed = True

        if (sensor_name == "MaxDistanceChanged_current"):
            self.max_distance_changed = True

        if (sensor_name == "NoFlyOverMaxDistanceChanged_shouldNotFlyOver"):
            self.no_fly_over_max_distance_changed = True

        if (sensor_name == "MaxTiltChanged_current"):
            self.max_tilt_changed = True

        if (sensor_name == "MaxPitchRollRotationSpeedChanged_current"):
            self.max_pitch_roll_rotation_speed_changed = True

        if (sensor_name == "MaxVerticalSpeedChanged_current"):
            self.max_vertical_speed_changed = True

        if (sensor_name == "MaxRotationSpeedChanged_current"):
            self.max_rotation_speed_changed = True

        if (sensor_name == "HullProtectionChanged_present"):
            self.hull_protection_changed = True

        if (sensor_name == "OutdoorChanged_present"):
            self.outdoor_mode_changed = True

        if (sensor_name == "BatteryStateChanged_battery_percent"):
            self.battery = sensor_value

        if (sensor_name == "PictureFormatChanged_type"):
            self.picture_format_changed = True

        if (sensor_name == "AutoWhiteBalanceChanged_type"):
            self.auto_white_balance_changed = True

        if (sensor_name == "ExpositionChanged_value"):
            self.exposition_changed = True

        if (sensor_name == "SaturationChanged_value"):
            self.saturation_changed = True

        if (sensor_name == "TimelapseChanged_enabled"):
            self.timelapse_changed = True

        if (sensor_name == "VideoStabilizationModeChanged_mode"):
            self.video_stabilization_changed = True

        if (sensor_name == "VideoRecordingModeChanged_mode"):
            self.video_recording_changed = True

        if (sensor_name == "VideoFramerateChanged_framerate"):
            self.video_framerate_changed = True

        if (sensor_name == "VideoResolutionsChanged_type"):
            self.video_resolutions_changed = True

        # call the user callback if it isn't None
        if (self.user_callback_function is not None):
            self.user_callback_function(self.user_callback_function_args)

    def __str__(self):
        str = "Bebop sensors: %s" % self.sensors_dict
        return str


class Bebop2:
    """
    Manages communication with the Bebop drone, handling video capture,
    sensor updates, and camera control via ROS.
    """

    def __init__(self, drone_type: str = "Bebop2",
                 ip_address: str = "192.168.0.202",
                 frequency: int = 30):
        """
        Initializes the Bebop2 instance with specified drone type and IP
        address.

        :param drone_type: Type of drone, default is 'Bebop2'.
        :param ip_address: The drone's IP address, default is '192.168.0.202'.
        """
        self.drone_type = drone_type
        self.ip_address = ip_address
        self.frequency = frequency

        if not self._check_connection():
            rospy.logerr("Bebop2 not connected to the network.")
            exit(1)

        self.state_map = self._initialize_state_map()

        # Initialize ros-drone components
        self.camera = DroneCamera(drone_type, frequency)
        self.control = DroneControl(drone_type, frequency)
        self.gps = GPSStateManager(drone_type, frequency)
        self.health = HealthMonitor(drone_type, frequency)
        self.params = ParameterManager(drone_type, frequency)
        self.media = DroneMedia(drone_type, frequency)
        self.sensors = DroneSensors(drone_type)
        self.states = FlightStateManager(drone_type, frequency)

        self.sensor_parser = DroneSensors(drone_type, frequency)

        self.command_interval = 1.0 / frequency
        self.last_command_time = rospy.get_time()
        self.main_dir = os.path.dirname(__file__)

    # Private Methods

    @staticmethod
    def _initialize_state_map() -> dict:
        """
        Initializes the state mapping for the drone.

        :return: A dictionary mapping state codes to state descriptions.
        """
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
        Checks if enough time has passed since the last command.

        :return: True if the command interval has passed; False otherwise.
        """
        current_time = rospy.get_time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            return True
        return False

    # General Management Methods

    def set_user_sensor_callback(self, function: Callable, args: tuple
                                 ) -> None:
        """
        Set the (optional) user callback function for sensors.  Every time a
        sensor is updated, it calls this function.

        :param function: name of the function
        :param args: tuple of arguments to the function
        :return: nothing
        """
        self.sensors.set_user_callback_function(function, args)

    def update_sensors(self, raw_data, *args) -> None:
        """
        Updates the sensor values from the raw data received.

        :param raw_data: Raw sensor data received from the drone.
        """
        sensor_list = self.sensor_parser.extract_sensor_values(raw_data)
        if sensor_list is not None:
            for sensor in sensor_list:
                (sensor_name, sensor_value, sensor_enum, header_tuple) = sensor
                if sensor_name is not None:
                    self.sensors.update(sensor_name, sensor_value, sensor_enum)
                else:
                    print("Error empty sensor")

    def connected(self) -> bool:
        """
        Checks if the drone is connected to the network.

        :return: True if the drone is connected; False otherwise.
        """
        response = os.system(f"ping -c 1 {self.ip_address}")
        return response == 0

    def disconnect(self):
        pass

    # Drone Control Methods

    def _check_emergency_state(self) -> bool:
        """Checks if the drone is in an emergency state."""
        return self.sensors.flying_state == self.state_map['E']

    def _check_landed_state(self) -> bool:
        """Checks if the drone is in a landed state."""
        return self.sensors.flying_state == self.state_map['L']

    def takeoff(self) -> None:
        """
        Sends a takeoff command if the drone is landed and not in emergency.
        """
        if self._check_emergency_state():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if self._check_landed_state():
            self.control.takeoff()
            self.sensors.flying_state = self.state_map['H']
            rospy.loginfo("Bebop is taking off.")
        else:
            rospy.loginfo("Bebop is already flying or busy.")

    def safe_takeoff(self, timeout: int = 5) -> None:
        """
        Safely takes off, ensuring the drone reaches the hovering state within
        the timeout.

        :param timeout: Maximum time to wait for the drone to take off (
                        seconds).
        """
        if self._check_emergency_state():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if self._check_landed_state():
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < timeout:
                self.control.takeoff()
                rospy.loginfo("Bebop is taking off.")
                self.sensors.flying_state = self.state_map['T']
                rospy.sleep(0.1)

                if self.sensors.get_altitude() > 0.5: #
                    self.sensors.flying_state = self.state_map['H']
                    rospy.loginfo("Bebop is hovering.")
                    return
        else:
            rospy.loginfo("Bebop is already flying or busy.")

    def _check_hovering_state(self) -> bool:
        """Checks if the drone is hovering."""
        return self.sensors.flying_state == self.state_map['H']

    def land(self) -> None:
        """
        Sends a land command if the drone is hovering and not in emergency.
        """
        if self._check_emergency_state():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if self._check_hovering_state():
            self.control.land()
            self.sensors.flying_state = self.state_map['L']
            rospy.loginfo("Bebop is landing.")
        else:
            rospy.loginfo("Bebop is already landed or busy.")

    def safe_land(self, timeout: int = 5) -> None:
        """
        Safely lands, ensuring the drone lands within the timeout.

        :param timeout: Maximum time to wait for the drone to land (seconds).
        """
        if self._check_emergency_state():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if self._check_hovering_state():
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < timeout:
                self.control.land()
                rospy.loginfo("Bebop is landing.")
                self.sensors.flying_state = self.state_map['l']
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

    def _ensure_fly_command_in_range(self, value: float) -> float:
        """
        Normalizes percentage input to a range between -1.0 and 1.0.

        :param value: The input percentage (-100 to 100).
        :return: Normalized value (-1.0 to 1.0).
        """
        if value < -100:
            return -1.0
        if value > 100:
            return 1.0
        return value / 100.0

    def fly_direct(self, roll: float, pitch: float, yaw: float,
                   vertical: float, duration: float) -> None:
        """
        Sends a direct flight command to the drone.

        :param roll: Roll value (-100 to 100).
        :param pitch: Pitch value (-100 to 100).
        :param yaw: Yaw value (-100 to 100).
        :param vertical: Vertical value (-100 to 100).
        :param duration: Duration of the flight command.
        """
        if self._check_emergency_state():
            rospy.loginfo("Bebop is in emergency mode.")
            return
        
        if not self._check_hovering_state():
            rospy.logwarn("Bebop must be hovering to fly.")
            return
        
        roll = self._ensure_fly_command_in_range(roll)
        pitch = self._ensure_fly_command_in_range(pitch)
        yaw = self._ensure_fly_command_in_range(yaw)
        vertical = self._ensure_fly_command_in_range(vertical)

        if duration == 0:
            self.control.move(roll, pitch, vertical, yaw)
            rospy.loginfo(f"Direct flight command sent: roll={roll}, pitch={pitch}, "
                          f"yaw={yaw}, vertical={vertical}.")
        else:
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < duration:
                self.control.move(roll, pitch, vertical, yaw)
                rospy.loginfo(f"Direct flight command sent: roll={roll}, pitch={pitch}, "
                              f"yaw={yaw}, vertical={vertical}.")
                rospy.sleep(0.1)
            self.control.move(0, 0, 0, 0)
            rospy.loginfo("Direct flight command ended.")

    def flip(self, direction: str) -> None:
        """
        Performs a flip in the specified direction.

        :param direction: Direction for the flip.
        """
        if self._check_emergency_state():
            rospy.loginfo("Bebop is in emergency mode.")
            return

        if not self._check_hovering_state():
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
        Move the drone relative to its current position based on distances
        provided along the X, Y, Z axes, and a rotation angle.

        :param dx: Distance to move along the X-axis (in meters, positive is
            forward).
        :param dy: Distance to move along the Y-axis (in meters, positive is
            right).
        :param dz: Distance to move along the Z-axis (in meters, positive is
            down).
        :param dr: Rotation in radians around the Z-axis (positive is
            clockwise).
        :param velocity: The movement velocity (default: 0.1).
        """
        if self._check_emergency_state():
            rospy.loginfo("Drone is in emergency mode.")
            return

        # Desired position and current position
        desired_position = np.array([dx, dy, dz, dr])
        current_position = np.array(self.sensors.sensors_dict['X'])[[0, 1, 2, 5]]  # Assuming [X, Y, Z, Rotation]
        error = desired_position - current_position

        # Loop until error in any dimension is reduced below 0.1
        while (np.abs(error) >= 0.1).any():
            movement_vector = np.zeros(4)
            if np.abs(error[0]) > 0.1:
                movement_vector[0] = np.sign(error[0]) * velocity
            if np.abs(error[1]) > 0.1:
                movement_vector[1] = np.sign(error[1]) * velocity
            if np.abs(error[2]) > 0.1:
                movement_vector[2] = np.sign(error[2]) * velocity
            if np.abs(error[3]) > 0.1:
                movement_vector[3] = np.sign(error[3]) * velocity

            self.control.move(*movement_vector)
            error = desired_position - np.array(self.sensors.sensors_dict['X'])[[0, 1, 2, 5]]  # Recalculate error

        # Stop movement after reaching the target
        self.control.move(0.0, 0.0, 0.0, 0.0)
        rospy.loginfo(f"Moved drone by dx={dx}, dy={dy}, dz={dz}, dr={dr}.")
        rospy.loginfo("Drone has reached the desired position.")

    # Camera Control Methods

    def capture_frame(self, subscriber: str = 'compressed'
                      ) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Captures the latest image frame from the camera.

        :param subscriber: The subscriber to read from (default: 'compressed').
        :return: Tuple containing success flag and the image data.
        """
        success = self.camera.success_flags.get(subscriber, False)
        image = self.camera.image_data.get(subscriber)
        return success, image

    def release_camera(self) -> None:
        """Releases camera resources and resets internal state."""
        self.camera.success_flags = {
            key: False for key in self.camera.success_flags}
        self.camera.image_data = {key: None for key in self.camera.image_data}
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
        filename = os.path.join(self.main_dir, 'images', 'snapshot_0000.png')
        # verifca se o arquivo existe, se sim, incrementa o número
        if os.path.exists(filename):
            i = 1
            while os.path.exists(filename):
                filename = os.path.join(self.main_dir, 'images',
                                        f'snapshot_{str(i).zfill(4)}.png')
                i += 1
        self.camera._save_image(frame, filename)

    def set_exposition(self, exposure_value: float) -> None:
        """
        Adjusts the camera's exposure setting.

        :param exposure_value: Exposure value to set (-3.0 to 3.0).
        """
        self.camera.adjust_exposure(exposure_value)
