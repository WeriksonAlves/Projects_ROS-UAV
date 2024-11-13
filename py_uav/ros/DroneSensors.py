"""
Sensors Module: Manages sensor data for Bebop2, including GPS, attitude,
speed, altitude, battery, and WiFi signal.

ROS Topics (10):
    - /bebop/fix (GPS data)
    - /bebop/joint_states
    - /bebop/odom
    - /bebop/states/ardrone3/PilotingState/AltitudeChanged
    - /bebop/states/ardrone3/PilotingState/AttitudeChanged
    - /bebop/states/ardrone3/PilotingState/PositionChanged
    - /bebop/states/ardrone3/PilotingState/SpeedChanged
    - /bebop/states/ardrone3/PilotingState/FlyingStateChanged
    - /bebop/states/common/CommonState/BatteryStateChanged
    - /bebop/states/common/CommonState/WifiSignalChanged
"""

import math
import rospy
from ..interfaces.RosCommunication import RosCommunication
from bebop_msgs.msg import (
    Ardrone3PilotingStateAltitudeChanged,
    Ardrone3PilotingStateAttitudeChanged,
    Ardrone3PilotingStatePositionChanged,
    Ardrone3PilotingStateSpeedChanged,
    Ardrone3PilotingStateFlyingStateChanged,
    CommonCommonStateBatteryStateChanged,
    CommonCommonStateWifiSignalChanged,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, NavSatFix
from typing import Any, Dict, List, Callable


class DroneSensors(RosCommunication):
    """
    Manages and updates Bebop2's sensor data via ROS topics, including
    odometry, GPS, altitude, attitude, speed, battery level, and WiFi signal.
    """

    _instance = None  # Instância singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(DroneSensors, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes the DroneSensors class with ROS subscribers for each drone
        sensor.

        :param drone_type: Type of the drone.
        :param frequency: Sensor update frequency in Hz (default: 30 Hz).
        """
        if hasattr(self, '_initialized') and self._initialized:
            return

        super().__init__(drone_type, frequency)
        self.sensor_manager = SensorDataManager(update_interval=1 / frequency)

        self._initialize_subscribers()
        rospy.loginfo(f"Sensors initialized for {drone_type}.")

        self._initialized = True

    def _initialize_publishers(self) -> None:
        return super()._initialize_publishers()

    def _initialize_subscribers(self) -> None:
        """Configures ROS subscribers for each sensor topic."""
        topic_map = {
            '/bebop/fix': (NavSatFix, self._process_gps_position),
            '/bebop/joint_states': (JointState, self._process_joint_state),
            '/bebop/odom': (Odometry, self._process_odometry),
            '/bebop/states/ardrone3/PilotingState/AltitudeChanged':
                (Ardrone3PilotingStateAltitudeChanged, self._process_altitude),
            '/bebop/states/ardrone3/PilotingState/AttitudeChanged':
                (Ardrone3PilotingStateAttitudeChanged, self._process_attitude),
            '/bebop/states/ardrone3/PilotingState/PositionChanged':
                (Ardrone3PilotingStatePositionChanged, self._process_position),
            '/bebop/states/ardrone3/PilotingState/SpeedChanged':
                (Ardrone3PilotingStateSpeedChanged, self._process_speed),
            '/bebop/states/ardrone3/PilotingState/FlyingStateChanged':
                (Ardrone3PilotingStateFlyingStateChanged,
                 self._process_flying_state),
            '/bebop/states/common/CommonState/BatteryStateChanged':
                (CommonCommonStateBatteryStateChanged,
                 self._process_battery_level),
            '/bebop/states/common/CommonState/WifiSignalChanged':
                (CommonCommonStateWifiSignalChanged,
                 self._process_wifi_signal),
        }
        for topic, (msg_type, callback) in topic_map.items():
            rospy.Subscriber(topic, msg_type, callback)

    # Sensor data processing methods
    def _process_odometry(self, odom: Odometry) -> None:
        """Processes odometry-based sensor data."""
        self.sensor_manager.update_sensor("position", odom,
                                          self._extract_position)
        self.sensor_manager.update_sensor("orientation", odom,
                                          self._extract_orientation)
        self.sensor_manager.update_sensor("speed_linear", odom,
                                          self._extract_linear_speed)
        self.sensor_manager.update_sensor("speed_angular", odom,
                                          self._extract_angular_speed)

    def _process_gps_position(self, gps_data: NavSatFix) -> None:
        """Processes GPS position data."""
        self.sensor_manager.update_sensor("gps_position", gps_data,
                                          self._extract_gps_position)

    def _process_joint_state(self, joint_state: JointState) -> None:
        """Processes joint state data."""
        self.sensor_manager.update_sensor("joint_state", joint_state,
                                          self._extract_joint_state)

    def _process_altitude(self,
                          altitude_data: Ardrone3PilotingStateAltitudeChanged
                          ) -> None:
        """Processes altitude data."""
        self.sensor_manager.update_sensor("altitude", altitude_data,
                                          lambda x: x.altitude)

    def _process_attitude(self,
                          attitude_data: Ardrone3PilotingStateAttitudeChanged
                          ) -> None:
        """Processes attitude data."""
        self.sensor_manager.update_sensor("attitude", attitude_data,
                                          self._extract_attitude)

    def _process_position(self,
                          position_data: Ardrone3PilotingStatePositionChanged
                          ) -> None:
        """Processes GPS-based position data."""
        self.sensor_manager.update_sensor("gps_position", position_data,
                                          self._extract_gps_position)

    def _process_speed(self, speed_data: Ardrone3PilotingStateSpeedChanged
                       ) -> None:
        """Processes speed data."""
        self.sensor_manager.update_sensor("speed", speed_data,
                                          self._extract_speed)

    def _process_flying_state(
            self, flying_state: Ardrone3PilotingStateFlyingStateChanged
            ) -> None:
        """Processes flying state data."""
        self.sensor_manager.update_sensor("flying_state", flying_state,
                                          lambda x: x.state)

    def _process_battery_level(
            self, battery_data: CommonCommonStateBatteryStateChanged) -> None:
        """Processes battery level data."""
        self.sensor_manager.update_sensor("battery_level", battery_data,
                                          lambda x: x.percent)

    def _process_wifi_signal(self,
                             wifi_data: CommonCommonStateWifiSignalChanged
                             ) -> None:
        """Processes WiFi signal strength data."""
        self.sensor_manager.update_sensor("wifi_signal", wifi_data,
                                          lambda x: x.rssi)

    # Data extraction methods
    def _extract_position(self, odom: Odometry) -> List[float]:
        return [odom.pose.pose.position.x, odom.pose.pose.position.y,
                odom.pose.pose.position.z]

    def _extract_orientation(self, odom: Odometry) -> List[float]:
        x, y = odom.pose.pose.orientation.x, odom.pose.pose.orientation.y
        z, w = odom.pose.pose.orientation.z, odom.pose.pose.orientation.w
        return [
            math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(2 * (w * y - z * x)),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        ]

    def _extract_linear_speed(self, odom: Odometry) -> List[float]:
        return [odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                odom.twist.twist.linear.z]

    def _extract_angular_speed(self, odom: Odometry) -> List[float]:
        return [odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                odom.twist.twist.angular.z]

    def _extract_gps_position(self, gps_data: NavSatFix) -> List[float]:
        return [gps_data.latitude, gps_data.longitude, gps_data.altitude]

    def _extract_joint_state(self, joint_state: JointState) -> List[float]:
        return joint_state.position

    def _extract_attitude(self,
                          attitude_data: Ardrone3PilotingStateAttitudeChanged
                          ) -> List[float]:
        return [attitude_data.roll, attitude_data.pitch, attitude_data.yaw]

    def _extract_speed(self, speed_data: Ardrone3PilotingStateSpeedChanged
                       ) -> List[float]:
        return [speed_data.speedX, speed_data.speedY, speed_data.speedZ]

    def get_processed_sensor_data(self) -> Dict[str, Any]:
        """
        Retrieves processed sensor data as a dictionary.

        :return: Dictionary of sensor data [position, orientation,
                 speed_linear, speed_angular, gps_position, joint_state,
                 altitude, attitude, speed, flying_state, battery_level,
                 wifi_signal].
        """
        return self.sensor_manager.get_data()


class SensorDataManager:
    """
    Manages sensor data storage, applying data conversion and update
    intervals for Bebop2's sensors.
    """

    def __init__(self, update_interval: float):
        """
        Initializes SensorDataManager with an update interval for sensor data.

        :param update_interval: Time interval for updating sensor data.
        """
        self.update_interval = update_interval
        self.data = {}
        self.timestamps = {}

    def should_update(self, sensor_name: str) -> bool:
        """Checks if enough time has passed to update a given sensor."""
        current_time = rospy.get_time()
        last_update = self.timestamps.get(sensor_name)
        if last_update is None or (current_time - last_update
                                   ) >= self.update_interval:
            self.timestamps[sensor_name] = current_time
            return True
        return False

    def update_sensor(self, sensor_name: str, data: Any, converter: Callable[
            [Any], Any]) -> None:
        """
        Updates sensor data if the update interval requirement is met and
        applies data conversion.
        """
        if self.should_update(sensor_name):
            self.data[sensor_name] = converter(data)

    def get_data(self) -> Dict[str, Any]:
        """Retrieves all stored sensor data with converted values."""
        return self.data
