# File: DroneMedia.py

This module contains the `DroneMedia` class, which is responsible for managing media-related operations on the drone. These operations include video recording, streaming state monitoring, MAVLink file playing state, and error tracking.

---

## Public Methods:

### `DroneMedia`

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton design pattern to ensure only one instance of `DroneMedia` is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, frequency: int = 30)`**
   - Initializes the `DroneMedia` instance by setting up publishers and subscribers for managing media operations, including video streaming and MAVLink file handling.
   - Parameters:
     - `drone_type`: Type of drone (e.g., "Bebop2").
     - `frequency`: The frequency for media command updates (default: 30 Hz).

3. **`_initialize_subscribers(self) -> None`**
   - Sets up ROS subscribers for monitoring media-related topics, such as video streaming state, MAVLink file playing state, and MAVLink play errors.

4. **`_initialize_publishers(self) -> None`**
   - Initializes a ROS publisher for controlling video recording commands (start/stop recording).

5. **`_time_to_update(self) -> bool`**
   - Checks if enough time has passed since the last media operation, ensuring that commands are not sent too frequently.
   - Returns `True` if it's time to send an update, `False` otherwise.

6. **`_video_state_callback(self, msg: Int32) -> None`**
   - Callback to handle changes in the video streaming state. It updates the `video_enabled` flag based on the message received (0 for disabled, 1 for enabled).

7. **`_mavlink_state_callback(self, msg: Int32) -> None`**
   - Callback to handle changes in the MAVLink file playing state. It updates the `mavlink_playing_state` variable based on the received message.

8. **`_mavlink_error_callback(self, msg: String) -> None`**
   - Callback to handle MAVLink play errors. It updates the `mavlink_error` variable and logs a warning message if an error occurs.

9. **`start_recording(self) -> None`**
   - Publishes a command to start video recording. It triggers the video recording process on the drone.

10. **`stop_recording(self) -> None`**
   - Publishes a command to stop video recording. It stops the video recording process on the drone.

11. **`get_video_state(self) -> bool`**
   - Retrieves the current state of video streaming. It returns `True` if video streaming is enabled, and `False` if it is disabled.

12. **`get_mavlink_playing_state(self) -> int`**
   - Retrieves the current MAVLink file playing state. It returns an integer representing the state of the MAVLink file (e.g., 0 for stopped, 1 for playing).

13. **`get_mavlink_error(self) -> str`**
   - Retrieves the latest MAVLink play error message, if any. It returns a string with the MAVLink play error details.

---

# Summary

- **`DroneMedia`** manages media operations for the drone, including controlling video recording, monitoring video streaming, and handling MAVLink file play states and errors.
- It subscribes to topics related to media state updates and publishes commands for starting and stopping video recording.
- The class uses a Singleton pattern to ensure that only one instance of `DroneMedia` exists and operates throughout the system.
- It provides methods to query the current state of video streaming, MAVLink play state, and any MAVLink play errors.