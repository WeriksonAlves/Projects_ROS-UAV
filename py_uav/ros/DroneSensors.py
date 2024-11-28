"""
Sensors Module: Manages sensor data for Bebop2, including altitude, attitude,
battery level, flying state, GPS position, ground truth, odmetry, position,
speed linear and WiFi signal.
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
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from typing import Any, Callable, Dict, List


class DroneSensors(RosCommunication):
    """
    Singleton class for managing and processing Bebop2 drone sensor data.
    Subscribes to ROS topics to update sensor readings.
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(DroneSensors, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes DroneSensors with ROS topic subscriptions.

        :param drone_type: Drone type ('bebop2' or 'gazebo').
        :param frequency: Frequency for sensor data updates (default: 30 Hz).
        """
        if getattr(self, "_initialized", False):
            return

        super().__init__(drone_type, frequency)
        self.sensor_manager = SensorDataManager(update_interval=1 / frequency)
        self.last_position = None
        self.last_time = None

        self._initialize_subscribers()
        rospy.loginfo(f"Sensors initialized for {drone_type}.")
        self._initialized = True

    def _initialize_publishers(self) -> None:
        return super()._initialize_publishers()

    def _initialize_subscribers(self) -> None:
        """Configures ROS subscribers for sensor topics based on drone type."""
        topic_map = self._get_topic_map()

        if not topic_map:
            rospy.logwarn(f"Unknown drone type: {self.drone_type}")
            return

        for topic, (msg_type, callback) in topic_map.items():
            rospy.Subscriber(topic, msg_type, callback)

    def _get_topic_map(self) -> Dict[str, Callable]:
        """Returns the topic map for the given drone type."""
        topics = {
            'bebop2': {
                '/bebop/states/ardrone3/PilotingState/AltitudeChanged':
                    (Ardrone3PilotingStateAltitudeChanged,
                     self._process_altitude),
                '/bebop/states/ardrone3/PilotingState/AttitudeChanged':
                    (Ardrone3PilotingStateAttitudeChanged,
                     self._process_attitude),
                '/bebop/states/common/CommonState/BatteryStateChanged':
                    (CommonCommonStateBatteryStateChanged,
                     self._process_battery_level),
                '/bebop/states/ardrone3/PilotingState/FlyingStateChanged':
                    (Ardrone3PilotingStateFlyingStateChanged,
                     self._process_flying_state),
                '/bebop/fix': (NavSatFix, self._process_gps_position),
                '/bebop/odom': (Odometry, self._process_odometry),
                '/bebop/states/ardrone3/PilotingState/PositionChanged':
                    (Ardrone3PilotingStatePositionChanged,
                     self._process_position),
                '/bebop/states/ardrone3/PilotingState/SpeedChanged':
                    (Ardrone3PilotingStateSpeedChanged,
                     self._process_speed_linear),
                '/bebop/states/common/CommonState/WifiSignalChanged':
                    (CommonCommonStateWifiSignalChanged,
                     self._process_wifi_signal),
            },
            'gazebo': {
                '/bebop2/odometry_sensor1/pose':
                    (Pose, self._process_general_info),
                '/bebop2/odometry_sensor1/odometry':
                    (Odometry, self._process_odometry),
                '/bebop2/ground_truth/odometry':
                    (Odometry, self._process_ground_truth),
            },
        }.get(self.drone_type.upper(), {})

        return topics

    # Sensor-specific data processing methods
    def _process_altitude(self, data: Ardrone3PilotingStateAltitudeChanged
                          ) -> None:
        self._update_sensor("altitude", data.altitude)

    def _process_attitude(self, data: Ardrone3PilotingStateAttitudeChanged
                          ) -> None:
        self._update_sensor("attitude", [data.roll, data.pitch, data.yaw])

    def _process_battery_level(self,
                               data: CommonCommonStateBatteryStateChanged
                               ) -> None:
        self._update_sensor("battery_level", data.percent)

    def _process_flying_state(self,
                              data: Ardrone3PilotingStateFlyingStateChanged
                              ) -> None:
        self._update_sensor("flying_state", data.state)

    def _process_gps_position(self, data: NavSatFix) -> None:
        self._update_sensor("gps_position", [data.latitude, data.longitude,
                                             data.altitude])

    def _process_odometry(self, data: Odometry) -> None:
        self._update_sensor("odometry", self._extract_odometry(data))

    def _process_position(self, data: Ardrone3PilotingStatePositionChanged
                          ) -> None:
        self._update_sensor("position", [data.latitude, data.longitude,
                                         data.altitude])

    def _process_speed_linear(self, data: Ardrone3PilotingStateSpeedChanged
                              ) -> None:
        self._update_sensor("speed_linear", [data.speedX, data.speedY,
                                             data.speedZ])

    def _process_wifi_signal(self, data: CommonCommonStateWifiSignalChanged
                             ) -> None:
        self._update_sensor("wifi_signal", data.rssi)

    # Gazebo-specific processing
    def _process_general_info(self, pose: Pose) -> None:
        self._update_sensor("pose", self._extract_pose(pose))

    def _process_ground_truth(self, data: Odometry) -> None:
        self._update_sensor("ground_truth", self._extract_odometry(data))

    # Utility methods
    def _update_sensor(self, name: str, value: Any) -> None:
        """Updates sensor data using SensorDataManager."""
        self.sensor_manager.update_sensor(name, value)

    def _extract_odometry(self, odom: Odometry) -> Dict[str, Any]:
        position = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                    odom.pose.pose.position.z]
        orientation = self._quaternion_to_euler(
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w
        )
        linear_speed = [odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                        odom.twist.twist.linear.z]
        angular_speed = [odom.twist.twist.angular.x,
                         odom.twist.twist.angular.y,
                         odom.twist.twist.angular.z]
        return {"position": position, "orientation": orientation,
                "linear_speed": linear_speed, "angular_speed": angular_speed}

    @staticmethod
    def _quaternion_to_euler(x: float, y: float, z: float, w: float
                             ) -> List[float]:
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return [roll, pitch, yaw]

    def get_processed_sensor_data(self) -> Dict[str, Any]:
        """
        Retrieves all processed sensor data.

        :return: Dictionary of sensor readings.
        """
        return self.sensor_manager.get_data()


class SensorDataManager:
    """
    Manages sensor data with controlled update intervals and conversion.
    """

    def __init__(self, update_interval: float):
        """
        Initializes the sensor data manager with the given update interval.

        :param update_interval: Time interval for sensor data updates.
        """
        self.update_interval = update_interval
        self.data = {}
        self.timestamps = {}

    def update_sensor(self, name: str, value: Any) -> None:
        """Updates a sensor value if the update interval has passed."""
        current_time = rospy.get_time()
        if name not in self.timestamps or (current_time - self.timestamps[name]
                                           ) >= self.update_interval:
            self.data[name] = value
            self.timestamps[name] = current_time

    def get_data(self) -> Dict[str, Any]:
        """Retrieves all current sensor data."""
        return self.data
