"""
Purpose: This class manages sensor data from the drone, including GPS,
         attitude, speed, and battery levels.

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
from bebop_msgs.msg import (Ardrone3PilotingStateAltitudeChanged,
                            Ardrone3PilotingStateAttitudeChanged,
                            Ardrone3PilotingStatePositionChanged,
                            Ardrone3PilotingStateSpeedChanged,
                            Ardrone3PilotingStateFlyingStateChanged,
                            CommonCommonStateBatteryStateChanged,
                            CommonCommonStateWifiSignalChanged)


class SensorDataManager:
    """
    Manages sensor data and update timestamps for Bebop2's sensors.
    """

    def __init__(self, update_interval: float):
        """
        Initialize the SensorDataManager with a specified update interval.

        :param update_interval: Time interval in seconds between updates.
        """
        self.update_interval = update_interval
        self.data = {}
        self.timestamps = {}

    def should_update(self, sensor_name: str) -> bool:
        """
        Check if enough time has passed to update the specified sensor.

        :param sensor_name: Name of the sensor to check.
        :return: True if the sensor should be updated, False otherwise.
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
        Update the data for the specified sensor.

        :param sensor_name: Name of the sensor to update.
        :param data: Sensor data to store.
        """
        self.data[sensor_name] = data

    def get_data(self) -> dict:
        """
        Get all stored sensor data.

        :return: Dictionary containing the latest sensor data.
        """
        return self.data


class Sensors:
    """
    Manages and updates the Bebop2's sensor data via ROS topics,
    including odometry, GPS, altitude, attitude, speed, battery level,
    and WiFi signal strength.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize the Sensors class and set up ROS subscribers
        to retrieve relevant sensor data.

        :param drone_type: The type of drone being used.
        :param frequency: Frequency for sensor data updates, in Hz (default:
                          30 Hz).
        """
        self.drone_type = drone_type
        self.sensor_manager = SensorDataManager(update_interval=1 / frequency)
        self._initialize_subscribers()
        rospy.loginfo(f"Sensors initialized for {self.drone_type}.")

    def _initialize_subscribers(self) -> None:
        """Sets up ROS subscribers for each relevant drone sensor topic."""
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

    # Callback methods for sensor data updates

    def _odom_callback(self, data: Odometry) -> None:
        """Update odometry data if necessary."""
        if self.sensor_manager.should_update("odom"):
            self.sensor_manager.update_data("odom", data)

    def _gps_callback(self, data: NavSatFix) -> None:
        """Update GPS data if necessary."""
        if self.sensor_manager.should_update("gps"):
            self.sensor_manager.update_data("gps", data)

    def _altitude_callback(self, data: Ardrone3PilotingStateAltitudeChanged
                           ) -> None:
        """Update altitude data if necessary."""
        if self.sensor_manager.should_update("altitude"):
            self.sensor_manager.update_data("altitude", data.altitude)

    def _attitude_callback(self, data: Ardrone3PilotingStateAttitudeChanged
                           ) -> None:
        """Update attitude data if necessary (roll, pitch, yaw)."""
        if self.sensor_manager.should_update("attitude"):
            self.sensor_manager.update_data("attitude", {
                'roll': data.roll,
                'pitch': data.pitch,
                'yaw': data.yaw
            })

    def _position_callback(self, data: Ardrone3PilotingStatePositionChanged
                           ) -> None:
        """Update position data if necessary (latitude, longitude, altitude)."""
        if self.sensor_manager.should_update("position"):
            self.sensor_manager.update_data("position", {
                'latitude': data.latitude,
                'longitude': data.longitude,
                'altitude': data.altitude
            })

    def _speed_callback(self, data: Ardrone3PilotingStateSpeedChanged) -> None:
        """Update speed data if necessary (vx, vy, vz)."""
        if self.sensor_manager.should_update("speed"):
            self.sensor_manager.update_data("speed", {
                'vx': data.speedX,
                'vy': data.speedY,
                'vz': data.speedZ
            })

    def _flying_state_callback(self,
                               data: Ardrone3PilotingStateFlyingStateChanged
                               ) -> None:
        """Update flying state data if necessary."""
        if self.sensor_manager.should_update("flying_state"):
            self.sensor_manager.update_data("flying_state", data.state)

    def _battery_callback(self, data: CommonCommonStateBatteryStateChanged
                          ) -> None:
        """Update battery level data if necessary."""
        if self.sensor_manager.should_update("battery_level"):
            self.sensor_manager.update_data("battery_level", data.percent)

    def _wifi_callback(self, data: CommonCommonStateWifiSignalChanged) -> None:
        """Update WiFi signal strength data if necessary."""
        if self.sensor_manager.should_update("wifi_signal"):
            self.sensor_manager.update_data("wifi_signal", data.rssi)

    # Public method to retrieve all sensor data

    def get_sensor_data(self) -> dict:
        """
        Retrieve the current sensor data in a structured dictionary.

        :return: Dictionary containing the latest sensor data.
        """
        return self.sensor_manager.get_data()
