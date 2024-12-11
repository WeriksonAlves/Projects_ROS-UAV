# File: DroneCommandManager.py

This module provides a `DroneCommandManager` class that acts as a facade for managing the drone's commands, including takeoff, landing, flips, and camera controls. It simplifies interaction with `DroneCamera`, `DroneControl`, and `DroneSensorManager`, providing unified access for handling drone operations.

---

## Public Methods:

1. **`__init__(self, drone_camera: DroneCamera, drone_control: DroneControl, sensor_manager: DroneSensorManager, show_log: bool = False) -> None`**
   - Initializes the `DroneCommandManager` instance with provided components for controlling the camera, drone, and sensors.
   - Parameters:
     - `drone_camera`: Object for handling camera operations.
     - `drone_control`: Object for handling drone commands.
     - `sensor_manager`: Object for handling sensor data.
     - `show_log`: If `True`, logs will be shown for commands executed.

2. **`reset(self) -> None`**
   - Resets the drone to its initial state, including resetting control and sensor states.
   - Logs the reset process if `show_log` is enabled.

3. **`takeoff(self) -> None`**
   - Commands the drone to take off.
   - Checks if the drone is in the "landed" state and updates the status to "hovering" once airborne.
   - Logs the process if `show_log` is enabled.

4. **`land(self) -> None`**
   - Commands the drone to land.
   - Updates the status to "landed" and "hovering" accordingly.
   - Logs the process if `show_log` is enabled.

5. **`safe_takeoff(self, height: float = 0.5, timeout: float = 3.0) -> bool`**
   - Attempts a safe takeoff, ensuring the drone reaches the specified height within the given timeout.
   - Returns `True` if successful, otherwise `False`.
   - Logs the process if `show_log` is enabled.

6. **`safe_land(self, height: float = 0.15, timeout: float = 3.0) -> None`**
   - Attempts a safe landing, ensuring the drone lands within the specified height and timeout.
   - Logs the process if `show_log` is enabled.

7. **`emergency_stop(self, height: float = 0.15) -> None`**
   - Executes an emergency stop by landing the drone immediately, regardless of current state.
   - Updates the status to "emergency".
   - Logs the process if `show_log` is enabled.

8. **`flip(self, direction: str) -> None`**
   - Commands the drone to perform a flip in the specified direction (left, right, forward, or backward).
   - Validates the current state before performing the flip.
   - Logs the process if `show_log` is enabled.

9. **`fly_direct(self, linear_x: float = 0.0, linear_y: float = 0.0, linear_z: float = 0.0, angular_z: float = 0.0, duration: float = 0.0) -> None`**
   - Commands the drone to move directly with specified velocities along the x, y, z axes, and angular velocity around the z-axis.
   - If `duration` is provided, the movement is time-limited.
   - Logs the process if `show_log` is enabled.

10. **`move_relative(self, delta_x: float = 0.0, delta_y: float = 0.0, delta_z: float = 0.0, delta_yaw: float = 0.0, power: int = 0.25, threshold: float = 0.05, rate: float = 1 / 30) -> None`**
    - Moves the drone in the specified relative direction based on the current position.
    - Uses `power` to control movement intensity and `threshold` to stop when the target position is reached.
    - Logs the process if `show_log` is enabled.

11. **`adjust_camera_orientation(self, tilt: float, pan: float) -> None`**
    - Adjusts the drone's camera orientation by specifying tilt and pan angles.
    - Logs the process if `show_log` is enabled.

12. **`adjust_camera_exposure(self, exposure: float) -> None`**
    - Adjusts the camera's exposure setting, with a range from -3 to 3.
    - Logs the process if `show_log` is enabled.

13. **`release_camera(self) -> None`**
    - Releases the camera resources.
    - Logs the process if `show_log` is enabled.

14. **`save_snapshot(self, frame: np.ndarray, main_dir: str) -> None`**
    - Saves a snapshot from the drone's camera to the specified directory with a unique filename.
    - Uses `_generate_unique_snapshot_filename` for filename creation.

15. **`_generate_unique_snapshot_filename(self, main_dir: str) -> str`**
    - Generates a unique filename for saving snapshots.
    - The filename includes an incrementing counter to ensure uniqueness in the directory.
    - Creates the snapshot directory if it doesn't already exist.

---

This documentation provides a brief overview of each method in the `DroneCommandManager` class, explaining their functionality and parameters. The class abstracts and manages several key operations of the drone, including movement, control, and camera management.