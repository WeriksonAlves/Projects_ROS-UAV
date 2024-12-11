# File: DroneManagers.py

This module contains three classes: `ParameterManager`, `GPSStateManager`, and `HealthMonitor`. Each class is responsible for managing different aspects of the drone’s state via ROS topics. These include handling parameter descriptions and updates, monitoring GPS satellite count, and tracking overheat status.

---

## Public Methods:

### `ParameterManager`

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton design pattern to ensure only one instance of `ParameterManager` is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, frequency: int = 30)`**
   - Initializes the `ParameterManager` instance with ROS subscribers for parameter topics and sets up the required parameters.
   - Parameters:
     - `drone_type`: Type of drone (e.g., "Bebop2").
     - `frequency`: The frequency for command intervals (default: 30 Hz).

3. **`_initialize_subscribers(self) -> None`**
   - Sets up ROS subscribers for parameter-related topics: `/bebop/bebop_driver/parameter_descriptions` and `/bebop/bebop_driver/parameter_updates`.

4. **`_initialize_publishers(self) -> None`**
   - Initializes publishers for the `ParameterManager`. This method does nothing in this class as it relies only on subscribers.

5. **`_is_time_to_command(self) -> bool`**
   - Checks if enough time has passed since the last command, ensuring that commands are not sent too frequently.

6. **`_parameter_description_callback(self, msg: ConfigDescription) -> None`**
   - Callback method to handle parameter descriptions when received via ROS.

7. **`_parameter_update_callback(self, msg: Config) -> None`**
   - Callback method to handle parameter updates when received via ROS.

8. **`get_parameter_descriptions(self) -> ConfigDescription`**
   - Retrieves the latest parameter descriptions received.

9. **`get_parameter_updates(self) -> Config`**
   - Retrieves the latest parameter updates received.

---

### `GPSStateManager`

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton design pattern to ensure only one instance of `GPSStateManager` is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, frequency: int = 30)`**
   - Initializes the `GPSStateManager` with a ROS subscriber to track the number of GPS satellites.
   - Parameters:
     - `drone_type`: Type of drone (e.g., "Bebop2").
     - `frequency`: The frequency for command intervals (default: 30 Hz).

3. **`_initialize_subscribers(self) -> None`**
   - Sets up a ROS subscriber for the topic `/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged` to monitor satellite count updates.

4. **`_initialize_publishers(self) -> None`**
   - Initializes publishers for the `GPSStateManager`. This method does nothing in this class as it relies only on subscribers.

5. **`_is_time_to_command(self) -> bool`**
   - Checks if enough time has passed since the last command, ensuring that commands are not sent too frequently.

6. **`_gps_state_callback(self, msg: Ardrone3GPSStateNumberOfSatelliteChanged) -> None`**
   - Callback method to handle GPS satellite count updates when received via ROS.

7. **`get_satellite_count(self) -> int`**
   - Retrieves the current number of connected GPS satellites.

---

### `HealthMonitor`

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton design pattern to ensure only one instance of `HealthMonitor` is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, frequency: int = 30)`**
   - Initializes the `HealthMonitor` with a ROS subscriber to track the overheat status of the drone.
   - Parameters:
     - `drone_type`: Type of drone (e.g., "Bebop2").
     - `frequency`: The frequency for command intervals (default: 30 Hz).

3. **`_initialize_subscribers(self) -> None`**
   - Sets up a ROS subscriber for the topic `/bebop/states/common/OverHeatState/OverHeatChanged` to monitor overheat status updates.

4. **`_initialize_publishers(self) -> None`**
   - Initializes publishers for the `HealthMonitor`. This method does nothing in this class as it relies only on subscribers.

5. **`_is_time_to_command(self) -> bool`**
   - Checks if enough time has passed since the last command, ensuring that commands are not sent too frequently.

6. **`_overheat_state_callback(self, msg: CommonOverHeatStateOverHeatChanged) -> None`**
   - Callback method to handle overheat status updates when received via ROS.

7. **`is_overheating(self) -> bool`**
   - Checks if the drone is currently overheating.

---

# Summary

- **`ParameterManager`** handles real-time updates for drone parameters, ensuring that parameters are described and updated via ROS topics.
- **`GPSStateManager`** tracks the number of GPS satellites connected to the drone.
- **`HealthMonitor`** monitors the drone's health, particularly focusing on its overheat status.

Each class follows the Singleton design pattern to ensure that only one instance of each manager exists, with callbacks to handle updates from respective ROS topics. They also incorporate logic to ensure that commands are sent only at specified intervals, helping to manage the drone’s behavior efficiently.