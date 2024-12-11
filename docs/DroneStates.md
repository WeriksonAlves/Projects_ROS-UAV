# File: DroneStates.py

This module defines the `FlightStateManager` class, which is responsible for managing and tracking flight state information for the Bebop drone. It tracks various states like flat trim, navigate home state, flight plan availability, and the state of flight plan components. The class subscribes to appropriate ROS topics to receive state updates and provides methods to check and retrieve the latest flight state information.

---

## Public Methods:

### `FlightStateManager`

1. **`__new__(cls, *args, **kwargs)`**
   - Implements the Singleton pattern to ensure that only one instance of the `FlightStateManager` class is created. If an instance already exists, it returns the existing one.

2. **`__init__(self, drone_type: str, frequency: int = 30)`**
   - Initializes the `FlightStateManager` instance and subscribes to ROS topics for flight state updates. It also sets the frequency for checking state updates and initializes state variables.
   - Parameters:
     - `drone_type`: The type of drone being used (e.g., "bebop2").
     - `frequency`: The frequency for checking state updates (default: 30 Hz).

3. **`_initialize_subscribers(self) -> None`**
   - Sets up ROS subscribers for the relevant flight state topics, such as flat trim, navigate home, flight plan availability, and component states.

4. **`_initialize_publishers(self) -> None`**
   - Initializes publishers for the drone's flight states. This method is inherited from the parent class but is not actively used in this implementation.

5. **`_initialize_state_variables(self) -> None`**
   - Initializes internal variables to track the current states for flat trim, navigate home, flight plan availability, and flight plan components.

6. **`_update_flat_trim(self, msg: Int32) -> None`**
   - Callback method that updates the flat trim state based on incoming ROS messages. It logs the updated flat trim state value.

7. **`_update_navigate_home(self, msg: Int32) -> None`**
   - Callback method that updates the navigate home state based on incoming ROS messages. It logs the updated navigate home state value.

8. **`_update_flight_plan_availability(self, msg: Int32) -> None`**
   - Callback method that updates the flight plan availability state based on incoming ROS messages. It logs the updated flight plan availability value.

9. **`_update_flight_plan_components(self, msg: String) -> None`**
   - Callback method that updates the flight plan components state based on incoming ROS messages. It logs the updated flight plan components.

10. **`check_state_updates(self) -> None`**
    - Checks and processes state updates based on the defined update interval. It ensures that the state updates are processed at a controlled frequency and can log additional information if needed.

11. **`get_flight_state(self) -> dict`**
    - Retrieves the current flight state information, including flat trim, navigate home state, flight plan availability, and flight plan components. Returns this information as a dictionary.

---

# Summary

- **`FlightStateManager`** is a singleton class responsible for managing flight state information from the Bebop drone.
- It subscribes to various ROS topics related to flat trim, navigate home, flight plan availability, and flight plan components, and updates internal state variables accordingly.
- The class provides methods to check for state updates and retrieve the latest flight state information.
