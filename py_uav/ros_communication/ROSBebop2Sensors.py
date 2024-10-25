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


class ROSBebop2Sensors:
    """
    Manages and updates the Bebop2's sensor data via ROS topics,
    including odometry, GPS, altitude, attitude, speed, battery level,
    and WiFi signal strength.
    """

    def __init__(self, drone_type: str, frequency: int = 30):
        """
        Initialize the ROSBebop2Sensors class and set up ROS subscribers
        to retrieve relevant sensor data.

        :param drone_type: The type of drone being used.
        :param frequency: Frequency for sensor data updates, in Hz (default:
                          30 Hz).
        """
        self.drone_type = drone_type
        self.update_interval = 1 / frequency

        # Sensor data storage
        self.sensor_data = {
            "odom": None,
            "gps": None,
            "altitude": None,
            "attitude": None,
            "position": None,
            "speed": None,
            "flying_state": None,
            "battery_level": None,
            "wifi_signal": None
        }

        # Timestamps for each topic's latest update
        self.sensor_timestamps = {
            "odom": None,
            "gps": None,
            "altitude": None,
            "attitude": None,
            "position": None,
            "speed": None,
            "flying_state": None,
            "battery_level": None,
            "wifi_signal": None
        }

        self._initialize_subscribers()
        rospy.loginfo(f"ROSBebop2Sensors initialized for {self.drone_type}.")

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

    def _time_to_update(self, sensor_name: str) -> bool:
        """
        Determine if enough time has elapsed since the last update for a given
        sensor.

        :param sensor_name: The name of the sensor to check.
        :return: True if the update interval has elapsed, False otherwise.
        """
        current_time = time.time()
        last_update_time = self.sensor_timestamps.get(sensor_name)

        if last_update_time is None or (current_time - last_update_time) >= self.update_interval:
            self.sensor_timestamps[sensor_name] = current_time
            return True
        return False

    # Callback methods

    def _odom_callback(self, data: Odometry) -> None:
        """Callback for odometry data."""
        if self._time_to_update("odom"):
            self.sensor_data["odom"] = data

    def _gps_callback(self, data: NavSatFix) -> None:
        """Callback for GPS data."""
        if self._time_to_update("gps"):
            self.sensor_data["gps"] = data

    def _altitude_callback(self, data: Ardrone3PilotingStateAltitudeChanged
                           ) -> None:
        """Callback for altitude data."""
        if self._time_to_update("altitude"):
            self.sensor_data["altitude"] = data.altitude

    def _attitude_callback(self, data: Ardrone3PilotingStateAttitudeChanged
                           ) -> None:
        """Callback for attitude data (roll, pitch, yaw)."""
        if self._time_to_update("attitude"):
            self.sensor_data["attitude"] = {
                'roll': data.roll,
                'pitch': data.pitch,
                'yaw': data.yaw
            }

    def _position_callback(self, data: Ardrone3PilotingStatePositionChanged
                           ) -> None:
        """Callback for position data (latitude, longitude, altitude)."""
        if self._time_to_update("position"):
            self.sensor_data["position"] = {
                'latitude': data.latitude,
                'longitude': data.longitude,
                'altitude': data.altitude
            }

    def _speed_callback(self, data: Ardrone3PilotingStateSpeedChanged) -> None:
        """Callback for speed data (vx, vy, vz)."""
        if self._time_to_update("speed"):
            self.sensor_data["speed"] = {
                'vx': data.speedX,
                'vy': data.speedY,
                'vz': data.speedZ
            }

    def _flying_state_callback(self,
                               data: Ardrone3PilotingStateFlyingStateChanged
                               ) -> None:
        """Callback for flying state data."""
        if self._time_to_update("flying_state"):
            self.sensor_data["flying_state"] = data.state

    def _battery_callback(self, data: CommonCommonStateBatteryStateChanged
                          ) -> None:
        """Callback for battery level data."""
        if self._time_to_update("battery_level"):
            self.sensor_data["battery_level"] = data.percent

    def _wifi_callback(self, data: CommonCommonStateWifiSignalChanged) -> None:
        """Callback for WiFi signal strength data."""
        if self._time_to_update("wifi_signal"):
            self.sensor_data["wifi_signal"] = data.rssi

    # Public methods

    def get_sensor_data(self) -> dict:
        """
        Retrieve the current sensor data in a structured dictionary.

        :return: Dictionary containing the latest sensor data: odometry, GPS,
                 altitude, attitude, position, speed, flying state,
                 battery level, and WiFi signal.
        """
        return self.sensor_data
