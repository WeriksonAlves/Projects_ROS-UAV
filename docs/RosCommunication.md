# File: RosCommunication.py

This module defines an abstract base class `RosCommunication`, which provides an interface for classes that interact with the Robot Operating System (ROS) for both publishing and subscribing to data. The class is intended to be extended by other classes to handle specific communication tasks for a particular drone.

---

## Public Methods:

1. **`__init__(self, drone_type: str, frequency: int = 30) -> None`**
   - Initializes the ROS communication settings, including the type of drone and the frequency at which commands are updated.
   - Parameters:
     - `drone_type`: Specifies the type of drone (e.g., "Bebop2").
     - `frequency`: The frequency (in Hz) at which commands should be sent (default: 30 Hz). This determines the interval between command updates.

2. **`_initialize_subscribers(self) -> None`**
   - Abstract method that must be implemented in subclasses.
   - This method is responsible for initializing the ROS subscribers, which handle incoming data from the drone or other sources.
   - The exact topics and subscription details must be defined in subclasses.

3. **`_initialize_publishers(self) -> None`**
   - Abstract method that must be implemented in subclasses.
   - This method is responsible for initializing the ROS publishers, which are used to send data to ROS.
   - The specific topics and publication details must be defined in subclasses.

---

This documentation provides a high-level overview of the `RosCommunication` class, which serves as an interface for managing ROS communication in drone control systems. It outlines the core functions that need to be implemented by subclasses for specific drone communication tasks.