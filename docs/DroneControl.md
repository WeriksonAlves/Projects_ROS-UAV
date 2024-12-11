# File: DroneControl.py

This module defines the `DroneControl` class, which manages the basic control operations for the Bebop drone. It allows for takeoff, landing, movement, flips, and autopilot commands through ROS topics. The class follows the Singleton design pattern, ensuring that only one instance of the `DroneControl` class is used throughout the system.

---

## Public Methods:

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton design pattern to ensure that only one instance of `DroneControl` is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, frequency: int = 30, show_log: bool = False)`**
   - Initializes the `DroneControl` instance by setting up necessary ROS publishers for controlling the drone. The class is initialized with a specified drone type, command frequency, and a logging option.
   - Parameters:
     - `drone_type`: Type of the drone (e.g., "Bebop2").
     - `frequency`: The frequency (in Hz) for issuing commands (default: 30 Hz).
     - `show_log`: Whether to enable logging for each published command (default: False).

3. **`_initialize_subscribers(self) -> None`**
   - Sets up ROS subscribers for receiving messages. Currently, no subscribers are required for the `DroneControl` class.

4. **`_initialize_publishers(self) -> dict`**
   - Initializes ROS publishers for various drone commands such as takeoff, landing, movement, flips, and autopilot control. It returns a dictionary of publisher instances for each command.

5. **`_publish_command(self, command: str, message=None) -> None`**
   - Publishes a given command to the corresponding ROS topic. If a message is provided, it is published; otherwise, an empty message is published. Optionally logs the command if `show_log` is enabled.

6. **`_is_time_to_command(self) -> bool`**
   - Checks if the minimum interval between commands has passed, ensuring commands are not sent too frequently. Returns `True` if sufficient time has elapsed, otherwise returns `False`.

7. **`takeoff(self) -> None`**
   - Commands the drone to take off. It checks if enough time has passed since the last command before sending the takeoff command.

8. **`land(self) -> None`**
   - Commands the drone to land. It checks if enough time has passed since the last command before sending the landing command.

9. **`reset(self) -> None`**
   - Commands the drone to reset. It checks if enough time has passed since the last command before sending the reset command.

10. **`move(self, linear_x: float = 0.0, linear_y: float = 0.0, linear_z: float = 0.0, angular_z: float = 0.0) -> None`**
    - Commands the drone to move with the specified velocities along the X, Y, and Z axes, as well as rotation around the Z-axis.
    - Parameters:
      - `linear_x`: Forward/backward velocity.
      - `linear_y`: Left/right velocity.
      - `linear_z`: Up/down velocity.
      - `angular_z`: Rotational velocity around the Z-axis.

11. **`flattrim(self) -> None`**
    - Commands the drone to perform a flat trim calibration to reset the drone's orientation and ensure stability. It checks if enough time has passed since the last command before sending the flat trim command.

12. **`flip(self, direction: str) -> None`**
    - Commands the drone to perform a flip in the specified direction. Valid directions are "forward", "backward", "left", and "right". If an invalid direction is specified, a warning is logged.
    - Parameters:
      - `direction`: The direction of the flip.

13. **`navigate_home(self, start: bool) -> None`**
    - Commands the drone to start or stop navigating back to its home position in an autopilot mission. 
    - Parameters:
      - `start`: `True` to start navigating home, `False` to stop.

14. **`pause(self) -> None`**
    - Pauses any ongoing autopilot mission, halting the current flight path or navigation task.

15. **`start_autoflight(self) -> None`**
    - Starts an autopilot mission, allowing the drone to follow a pre-programmed flight plan or trajectory.

16. **`stop_autoflight(self) -> None`**
    - Stops the current autopilot mission, interrupting the pre-programmed flight path or navigation task.

---

This documentation provides an overview of the `DroneControl` class, which is responsible for managing basic control operations for the Bebop drone. Each method handles a specific drone command, including movement, takeoff, landing, calibration, and autopilot-related commands. The class ensures that commands are not issued too frequently by enforcing a time interval between successive commands, and it allows for detailed control over the drone's behavior using ROS topics.