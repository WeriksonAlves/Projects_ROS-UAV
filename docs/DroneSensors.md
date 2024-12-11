# File: DroneSensors.py

This module defines the `DroneSensors` class, which is responsible for managing and processing various sensor data from the Bebop2 drone (or other drones in a simulation, such as Gazebo). The class subscribes to various ROS topics to receive updates about sensor readings, such as altitude, attitude, battery level, GPS position, odometry, WiFi signal strength, and more. It also processes this data and updates it through the `SensorDataManager`.

---

## Public Methods:

### `DroneSensors`

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton pattern to ensure that only one instance of the `DroneSensors` class is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, frequency: int = 30)`**
   - Initializes the `DroneSensors` instance and subscribes to appropriate ROS topics based on the specified drone type. It also initializes the sensor data manager and sets up the required frequency for sensor updates.
   - Parameters:
     - `drone_type`: The type of drone (e.g., "bebop2").
     - `frequency`: The frequency for sensor data updates (default: 30 Hz).

3. **`_initialize_publishers(self) -> None`**
   - Initializes the publishers for the drone sensors. This method is inherited from the parent class but is not actively used in this implementation.

4. **`_initialize_subscribers(self) -> None`**
   - Sets up ROS subscribers for the relevant sensor topics based on the drone type. It uses the `_get_topic_map()` method to determine which topics and corresponding callback functions should be subscribed to.

5. **`_get_topic_map(self) -> Dict[str, Callable]`**
   - Returns a dictionary that maps ROS topics to their respective message types and callback functions. This map varies based on the drone type (e.g., Bebop2 or Gazebo).

6. **`_process_altitude(self, data: Ardrone3PilotingStateAltitudeChanged) -> None`**
   - Processes altitude sensor data and updates the sensor value using the `SensorDataManager`.

7. **`_process_attitude(self, data: Ardrone3PilotingStateAttitudeChanged) -> None`**
   - Processes attitude sensor data (roll, pitch, yaw) and updates the sensor values.

8. **`_process_battery_level(self, data: CommonCommonStateBatteryStateChanged) -> None`**
   - Processes battery level data and updates the sensor value.

9. **`_process_flying_state(self, data: Ardrone3PilotingStateFlyingStateChanged) -> None`**
   - Processes flying state data (e.g., whether the drone is flying or not) and updates the sensor value.

10. **`_process_gps_position(self, data: NavSatFix) -> None`**
    - Processes GPS position data (latitude, longitude, altitude) and updates the sensor value.

11. **`_process_odometry(self, data: Odometry) -> None`**
    - Processes odometry data and updates the sensor value, including position, orientation, linear speed, and angular speed.

12. **`_process_position(self, data: Ardrone3PilotingStatePositionChanged) -> None`**
    - Processes position data (latitude, longitude, altitude) and updates the sensor value.

13. **`_process_speed_linear(self, data: Ardrone3PilotingStateSpeedChanged) -> None`**
    - Processes linear speed data (speed along X, Y, Z axes) and updates the sensor value.

14. **`_process_wifi_signal(self, data: CommonCommonStateWifiSignalChanged) -> None`**
    - Processes WiFi signal strength (RSSI) data and updates the sensor value.

15. **`_process_general_info(self, data: Pose) -> None`**
    - Processes general information in Gazebo, including attitude and position, and updates the corresponding sensor values.

16. **`_process_ground_truth(self, data: Odometry) -> None`**
    - Processes ground truth data in Gazebo (odometry) and updates the sensor value.

17. **`_update_sensor(self, name: str, value: Any) -> None`**
    - Updates the sensor data for a specific sensor using the `SensorDataManager`. It ensures that the data is updated according to the defined update interval.

18. **`_extract_odometry(self, odom: Odometry) -> Dict[str, Any]`**
    - Extracts position, orientation, linear speed, and angular speed data from odometry messages and returns it in a structured dictionary.

19. **`_quaternion_to_euler(x: float, y: float, z: float, w: float) -> List[float]`**
    - A static method that converts a quaternion (4D) orientation into Euler angles (roll, pitch, yaw) for easier interpretation and use.

20. **`get_processed_sensor_data(self) -> Dict[str, Any]`**
    - Retrieves all processed sensor data as a dictionary. This includes readings from all subscribed sensors, which are managed and updated through the `SensorDataManager`.

---

# Summary

- **`DroneSensors`** is a singleton class designed to manage and process various sensor data from the drone, including altitude, attitude, battery level, GPS position, odometry, WiFi signal strength, and more.
- It subscribes to appropriate ROS topics based on the drone type and processes the data using specialized methods for each sensor type.
- The sensor data is managed by the `SensorDataManager`, which ensures that updates happen at controlled intervals.
- The class provides an interface to retrieve the latest processed sensor data through the `get_processed_sensor_data()` method.