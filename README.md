# rospy_uav
**Python Interface for Autonomous Drone Control**

`rospy_uav` is a Python-based project designed to interface with Parrot drones, particularly the Bebop2, and simulate operations using Gazebo. Developed by Werikson Alves, this library serves as a foundation for autonomous drone navigation systems controlled via gesture recognition. It is ideal for developers, researchers, and hobbyists interested in exploring autonomous UAVs (Unmanned Aerial Vehicles) in real-world and simulated environments.

---

## **Features**
- **Real-time Vision Processing**: Capture and process video streams using `DroneVision` for tasks like gesture recognition and image-based navigation.
- **Gazebo Simulation Support**: Seamless integration with Gazebo for virtual testing and validation.
- **User-friendly Interface**: Simple APIs for managing drone movements, sensor updates, and vision tasks.
- **Safety-first Design**: Includes robust safety mechanisms like emergency landing and indoor-friendly parameter tuning.
- **Modular Design**: Easily extendable to support additional UAV models or advanced features.

---

## **Getting Started**

### **Installations**

#### Step 1 - Install `bebopautonomous`
This project leverages the features of [bebopautonomous](https://github.com/AutonomyLab/bebop_autonomy). Follow the installation guide available in its [documentation](https://bebop-autonomy.readthedocs.io/en/latest/installation.html).

#### Step 2 - Install `Parrot Bebop2 Gazebo and ROS Noetic`
The project also incorporates functionality from [iROS_drone](https://github.com/arnaldojr/iROS_drone/tree/noetic). Follow the setup instructions provided in the repository.

#### Step 3 - Set Up `rospy_uav`

1. **Navigate to the workspace folder and initialize the environment:**
   ```bash
   cd bebop_ws/
   . devel/setup.bash
   cd src/
   ```

2. **Clone the `rospy_uav` repository into a new folder:**
   ```bash
   mkdir env_rospy_uav
   cd env_rospy_uav/
   git clone https://github.com/WeriksonAlves/rospy_uav.git
   ```

3. **Create a virtual environment and install dependencies:**
   ```bash
   python3.9 -m venv venv
   source venv/bin/activate
   pip install -r rospy_uav/requirements.txt
   ```

4. **Write your custom code:**
   Create a new Python script (`main.py`) in the `env_rospy_uav` folder and implement your desired functionality. Refer to the provided examples for guidance.

**Note:** Ensure that your ROS (Robot Operating System) environment is properly configured and that all required packages are installed before proceeding.

---

## **Documentation**

Extensive documentation is provided to support your development:
- **Bebop2 topics using ROS:** Refer to the [pyparrot documentation](https://pyparrot.readthedocs.io), which shares many foundational concepts used in this project.
- **Classes and methods:** Explore the detailed documentation available in the `docs` folder.

---

## **Major Updates**
- **12/09/2024**: Initial release of the foundational library.
- **12/10/2024**: Added the requirements file for installing the necessary libraries.
- **12/11/2024**: Add documentation for ROS communication, drone vision, flight state management, and media operations.

---

## **Contributing**
We welcome contributions! Feel free to open issues or submit pull requests to enhance the project.

---

## **Programming and Using Drones Responsibly**
It is your responsibility to operate UAVs safely and responsibly. This library is intended for educational and research purposes. The developers are not liable for any damages, losses, or injuries incurred while using the software. Always follow local drone laws and guidelines.
