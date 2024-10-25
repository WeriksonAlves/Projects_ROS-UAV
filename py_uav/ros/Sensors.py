"""
Purpose: This module manages sensor data from the Bebop drone, including
GPS, attitude, speed, and battery levels.

Topics (9):
    /bebop/odom
    /bebop/fix (GPS data)
    /bebop/states/ardrone3/PilotingState/AltitudeChanged
    /bebop/states/ardrone3/PilotingState/AttitudeChanged
    /bebop/states/ardrone3/PilotingState/PositionChanged
    /bebop/states/ardrone3/PilotingState/SpeedChanged
    /bebop/states/ardrone3/PilotingState/FlyingStateChanged
    /bebop/states/common/CommonState/BatteryStateChanged
    /bebop/states/common/CommonState/WifiSignalChanged
"""

import rospy
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from bebop_msgs.msg import (
    Ardrone3PilotingStateAltitudeChanged,
    Ardrone3PilotingStateAttitudeChanged,
    Ardrone3PilotingStatePositionChanged,
    Ardrone3PilotingStateSpeedChanged,
    Ardrone3PilotingStateFlyingStateChanged,
    CommonCommonStateBatteryStateChanged,
    CommonCommonStateWifiSignalChanged,
)


class Sensors:
    """
    Manages and updates the Bebop2's sensor data via ROS topics, including
    odometry, GPS, altitude, attitude, speed, battery level, and WiFi signal.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize the Sensors class, setting up ROS subscribers.

        :param drone_type: The type of the drone.
        :param frequency: Sensor update frequency in Hz (default: 30 Hz).
        """
        self.drone_type = drone_type
        self.sensor_manager = SensorDataManager(update_interval=1 / frequency)
        self._initialize_subscribers()
        rospy.loginfo(f"Sensors initialized for {self.drone_type}.")

    def _initialize_subscribers(self) -> None:
        """Set up ROS subscribers for each drone sensor topic."""
        rospy.Subscriber('/bebop/odom', Odometry, self._odom_callback)
        rospy.Subscriber('/bebop/fix', NavSatFix, self._gps_callback)
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

    # Sensor callback methods
    def _odom_callback(self, data: Odometry) -> None:
        """Update odometry data if due."""
        if self.sensor_manager.should_update("odom"):
            self.sensor_manager.update_data("odom", data)

    def _gps_callback(self, data: NavSatFix) -> None:
        """Update GPS data if due."""
        if self.sensor_manager.should_update("gps"):
            self.sensor_manager.update_data("gps", data)

    def _altitude_callback(self, data: Ardrone3PilotingStateAltitudeChanged
                           ) -> None:
        """Update altitude data if due."""
        if self.sensor_manager.should_update("altitude"):
            self.sensor_manager.update_data("altitude", data.altitude)

    def _attitude_callback(self, data: Ardrone3PilotingStateAttitudeChanged
                           ) -> None:
        """Update attitude (roll, pitch, yaw) data if due."""
        if self.sensor_manager.should_update("attitude"):
            attitude_data = {'roll': data.roll, 'pitch': data.pitch,
                             'yaw': data.yaw}
            self.sensor_manager.update_data("attitude", attitude_data)

    def _position_callback(self, data: Ardrone3PilotingStatePositionChanged
                           ) -> None:
        """Update position data (latitude, longitude, altitude) if due."""
        if self.sensor_manager.should_update("position"):
            position_data = {'latitude': data.latitude,
                             'longitude': data.longitude,
                             'altitude': data.altitude}
            self.sensor_manager.update_data("position", position_data)

    def _speed_callback(self, data: Ardrone3PilotingStateSpeedChanged) -> None:
        """Update speed data (vx, vy, vz) if due."""
        if self.sensor_manager.should_update("speed"):
            speed_data = {'vx': data.speedX, 'vy': data.speedY,
                          'vz': data.speedZ}
            self.sensor_manager.update_data("speed", speed_data)

    def _flying_state_callback(self,
                               data: Ardrone3PilotingStateFlyingStateChanged
                               ) -> None:
        """Update flying state data if due."""
        if self.sensor_manager.should_update("flying_state"):
            self.sensor_manager.update_data("flying_state", data.state)

    def _battery_callback(self, data: CommonCommonStateBatteryStateChanged
                          ) -> None:
        """Update battery level data if due."""
        if self.sensor_manager.should_update("battery_level"):
            self.sensor_manager.update_data("battery_level", data.percent)

    def _wifi_callback(self, data: CommonCommonStateWifiSignalChanged) -> None:
        """Update WiFi signal strength data if due."""
        if self.sensor_manager.should_update("wifi_signal"):
            self.sensor_manager.update_data("wifi_signal", data.rssi)

    def get_sensor_data(self) -> dict:
        """
        Retrieve the latest sensor data.

        :return: A dictionary with current sensor readings.
        """
        return self.sensor_manager.get_data()


class SensorDataManager:
    """
    Manages the sensor data and timestamps for Bebop2's sensors, controlling
    the update frequency.
    """

    def __init__(self, update_interval: float):
        """
        Initialize the sensor data manager with a specific update interval.

        :param update_interval: Minimum time interval between updates in
                                seconds.
        """
        self.update_interval = update_interval
        self.data = {}
        self.timestamps = {}

    def should_update(self, sensor_name: str) -> bool:
        """
        Determine if the sensor data should be updated based on the interval.

        :param sensor_name: Name of the sensor.
        :return: True if enough time has passed, False otherwise.
        """
        current_time = time.time()
        last_update = self.timestamps.get(sensor_name)
        if last_update is None or (current_time - last_update
                                   ) >= self.update_interval:
            self.timestamps[sensor_name] = current_time
            return True
        return False

    def update_data(self, sensor_name: str, data) -> None:
        """
        Update the data for a specified sensor.

        :param sensor_name: Name of the sensor.
        :param data: Data to store for the sensor.
        """
        self.data[sensor_name] = data

    def get_data(self) -> dict:
        """
        Retrieve all stored sensor data.

        :return: Dictionary containing the latest sensor data.
        """
        return self.data
