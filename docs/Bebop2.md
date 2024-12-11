# Bebop2.py Documentation

## Overview
The `Bebop2` class provides an interface to control and manage the Parrot Bebop2 drone using ROS (Robot Operating System). It supports various functionalities such as flight control, sensor management, and camera operations.

## Public Methods

**Initialization:**
- `__init__(drone_type: str, ip_address: str, frequency: float = 30.0, show_log: bool = False)`  
  Initializes the Bebop2 instance by setting up ROS nodes, subsystems (camera, control, sensors, and managers), and a background thread for sensor updates.

---

**Sensor Callback Management:**
- `set_user_sensor_callback(callback: Callable, *args)`  
  Registers a user-defined callback function that will be executed on sensor updates.

- `trigger_callback()`  
  Executes the user-defined callback function with any provided arguments.

---

**Drone State and Utility Methods:**
- `smart_sleep(seconds: float)`  
  Pauses the program for a specified duration while ensuring ROS processes remain active.

- `update_sensors()`  
  Updates the sensor data using the `DroneSensorManager`.

- `check_connection() -> bool`  
  Checks whether the drone is connected to the network. Returns `True` if connected, otherwise `False`.

---

**Drone Control Methods:**
- `reset()`  
  Resets the drone to its initial state.

- `takeoff()`  
  Commands the drone to take off.

- `safe_takeoff(timeout: float = 3.0)`  
  Commands the drone to take off within a specified timeout. Falls back to emergency landing if takeoff fails.

- `land()`  
  Commands the drone to land.

- `safe_land(timeout: float = 3.0)`  
  Commands the drone to land safely within a timeout. Falls back to emergency landing if landing fails.

- `emergency_land()`  
  Performs an emergency stop and forces the drone to land.

- `is_landed() -> bool`  
  Checks if the drone is currently landed. Returns `True` if landed.

- `is_hovering() -> bool`  
  Checks if the drone is currently hovering. Returns `True` if hovering.

- `is_emergency() -> bool`  
  Checks if the drone is in an emergency state. Returns `True` if in an emergency.

- `fly_direct(linear_x: float, linear_y: float, linear_z: float, angular_z: float, duration: float)`  
  Commands the drone to fly directly with specified velocities for a given duration.

- `flip(direction: str)`  
  Commands the drone to perform a flip in a specified direction (e.g., 'left', 'right', 'forward', or 'backward').

- `move_relative(delta_x: float, delta_y: float, delta_z: float, delta_yaw: float, power: float = 0.25)`  
  Commands the drone to move to a relative position with the specified deltas and movement power.

---

**Camera Operations:**
- `release_camera()`  
  Releases the camera resources.

- `adjust_camera_orientation(tilt: float, pan: float, pitch_comp: float = 0.0, yaw_comp: float = 0.0)`  
  Adjusts the camera's orientation with optional compensation for pitch and yaw.

- `adjust_camera_exposure(exposure: float)`  
  Adjusts the camera's exposure setting within the allowed range.

- `take_snapshot()`  
  Captures a snapshot from the drone’s camera and saves the image.

- `read_image(subscriber: str = "compressed") -> Tuple[bool, np.ndarray]`  
  Reads an image from the drone's camera. Returns a tuple with a success flag and the image array.

- `camera_on() -> bool`  
  Checks whether the camera is operational. Returns `True` if operational.

---

**Helper Methods:**
- `_normalize_velocity(value: float, scale: int = 100, min_val: float = -1.0, max_val: float = 1.0) -> float`  
  Normalizes a velocity value to a specified range.

- `_execute_command(command: Callable, action_description: str) -> bool`  
  Executes a command safely, handling errors and logging the action’s outcome.

---

**Background Process:**
- `_sensor_update_loop()`  
  Background loop to periodically update sensor data at a fixed rate until ROS shuts down or a shutdown flag is triggered.

--- 

This documentation provides an overview of the functionalities offered by the `Bebop2` class, making it easier to understand and maintain the code.