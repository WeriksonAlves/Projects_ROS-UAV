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

import rospy
import time
from ..interfaces.RosCommunication import RosCommunication
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, NavSatFix
from bebop_msgs.msg import (
    Ardrone3PilotingStateAltitudeChanged,
    Ardrone3PilotingStateAttitudeChanged,
    Ardrone3PilotingStatePositionChanged,
    Ardrone3PilotingStateSpeedChanged,
    Ardrone3PilotingStateFlyingStateChanged,
    CommonCommonStateBatteryStateChanged,
    CommonCommonStateWifiSignalChanged,
)


class Sensors(RosCommunication):
    """
    Manages and updates Bebop2's sensor data via ROS topics, including
    odometry, GPS, altitude, attitude, speed, battery level, and WiFi signal
    strength.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initializes Sensors class with ROS subscribers for each drone sensor.

        :param drone_type: Type of the drone.
        :param frequency: Sensor update frequency in Hz (default: 30 Hz).
        """
        super().__init__(drone_type, frequency)
        self.sensor_manager = SensorDataManager(update_interval=1 / frequency)

        self._initialize_subscribers()
        rospy.loginfo(f"Sensors initialized for {self.drone_type}.")

    def _initialize_subscribers(self) -> None:
        """Sets up ROS subscribers for sensor data topics."""
        rospy.Subscriber('/bebop/fix', NavSatFix, self._gps_callback)
        rospy.Subscriber('/bebop/joint_states', JointState,
                         self._joint_states_callback)
        rospy.Subscriber('/bebop/odom', Odometry, self._odom_callback)
        rospy.Subscriber(
            '/bebop/states/ardrone3/PilotingState/AltitudeChanged',
            Ardrone3PilotingStateAltitudeChanged, self._altitude_callback)
        rospy.Subscriber(
            '/bebop/states/ardrone3/PilotingState/AttitudeChanged',
            Ardrone3PilotingStateAttitudeChanged, self._attitude_callback)
        rospy.Subscriber(
            '/bebop/states/ardrone3/PilotingState/PositionChanged',
            Ardrone3PilotingStatePositionChanged, self._position_callback)
        rospy.Subscriber(
            '/bebop/states/ardrone3/PilotingState/SpeedChanged',
            Ardrone3PilotingStateSpeedChanged, self._speed_callback)
        rospy.Subscriber(
            '/bebop/states/ardrone3/PilotingState/FlyingStateChanged',
            Ardrone3PilotingStateFlyingStateChanged,
            self._flying_state_callback)
        rospy.Subscriber(
            '/bebop/states/common/CommonState/BatteryStateChanged',
            CommonCommonStateBatteryStateChanged, self._battery_callback)
        rospy.Subscriber(
            '/bebop/states/common/CommonState/WifiSignalChanged',
            CommonCommonStateWifiSignalChanged, self._wifi_callback)

    def _initialize_publishers(self) -> None:
        return super()._initialize_publishers()

    # Sensor callback methods

    def _odom_callback(self, data: Odometry) -> None:
        """Updates odometry data when due."""
        self.sensor_manager.update_sensor_if_due("odom", data)

    def _gps_callback(self, data: NavSatFix) -> None:
        """Updates GPS data when due."""
        self.sensor_manager.update_sensor_if_due("gps", data)

    def _joint_states_callback(self, data: JointState) -> None:
        """Updates joint state data when due."""
        self.sensor_manager.update_sensor_if_due("joint_states", data)

    def _altitude_callback(self, data: Ardrone3PilotingStateAltitudeChanged
                           ) -> None:
        """Updates altitude data when due."""
        self.sensor_manager.update_sensor_if_due("altitude", data.altitude)

    def _attitude_callback(self, data: Ardrone3PilotingStateAttitudeChanged
                           ) -> None:
        """Updates attitude data (roll, pitch, yaw) when due."""
        attitude_data = {'roll': data.roll, 'pitch': data.pitch,
                         'yaw': data.yaw}
        self.sensor_manager.update_sensor_if_due("attitude", attitude_data)

    def _position_callback(self, data: Ardrone3PilotingStatePositionChanged
                           ) -> None:
        """Updates position data (latitude, longitude, altitude) when due."""
        position_data = {'latitude': data.latitude,
                         'longitude': data.longitude,
                         'altitude': data.altitude}
        self.sensor_manager.update_sensor_if_due("position", position_data)

    def _speed_callback(self, data: Ardrone3PilotingStateSpeedChanged) -> None:
        """Updates speed data (vx, vy, vz) when due."""
        speed_data = {'vx': data.speedX, 'vy': data.speedY, 'vz': data.speedZ}
        self.sensor_manager.update_sensor_if_due("speed", speed_data)

    def _flying_state_callback(self,
                               data: Ardrone3PilotingStateFlyingStateChanged
                               ) -> None:
        """Updates flying state when due."""
        self.sensor_manager.update_sensor_if_due("flying_state", data.state)

    def _battery_callback(self, data: CommonCommonStateBatteryStateChanged
                          ) -> None:
        """Updates battery level when due."""
        self.sensor_manager.update_sensor_if_due("battery_level", data.percent)

    def _wifi_callback(self, data: CommonCommonStateWifiSignalChanged) -> None:
        """Updates WiFi signal strength when due."""
        self.sensor_manager.update_sensor_if_due("wifi_signal", data.rssi)

    def get_raw_sensor_data(self) -> dict:
        """
        Retrieves the latest sensor data.

        :return: Dictionary with current sensor readings.
        """
        return self.sensor_manager.get_data()


class SensorDataManager:
    """
    Manages sensor data storage and update intervals for Bebop2's sensors.
    """

    def __init__(self, update_interval: float):
        """
        Initializes the data manager with a specified update interval.

        :param update_interval: Minimum time interval between updates in
        seconds.
        """
        self.update_interval = update_interval
        self.data = {}
        self.timestamps = {}

    def should_update(self, sensor_name: str) -> bool:
        """
        Checks if enough time has passed to update a given sensor.

        :param sensor_name: Name of the sensor.
        :return: True if time interval requirement is met, False otherwise.
        """
        current_time = rospy.get_time()
        last_update = self.timestamps.get(sensor_name)
        if last_update is None or (current_time - last_update
                                   ) >= self.update_interval:
            self.timestamps[sensor_name] = current_time
            return True
        return False

    def update_sensor_if_due(self, sensor_name: str, data) -> None:
        """
        Updates sensor data if the update interval requirement is met.

        :param sensor_name: Name of the sensor.
        :param data: Sensor data to be stored.
        """
        if self.should_update(sensor_name):
            self.data[sensor_name] = data

    def get_data(self) -> dict:
        """
        Retrieves all stored sensor data.

        :return: Dictionary with the latest sensor data.
        """
        return self.data
