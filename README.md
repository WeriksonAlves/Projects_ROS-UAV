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

### **Installation**
Follow these steps to set up `py_uav`:

1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/py_uav.git
   cd py_uav
   ```

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Ensure your ROS (Robot Operating System) environment is configured and the necessary packages are installed.

Note: This project builds on the capabilities of [bebopautonomous](https://github.com/amymcgovern/bebopautonomous) and [pyparrot](https://github.com/amymcgovern/pyparrot).

---

## **Documentation**
Comprehensive documentation is available, covering installation, API references, and tutorials:
- **Documentation**: [Visit pyparrot's documentation](https://pyparrot.readthedocs.io) (many principles are shared).

---

## **Major Updates**
- **12/02/2024**: Initial release of the foundational library.

---

## **Contributing**
We welcome contributions! Feel free to open issues or submit pull requests to enhance the project.

---

## **Programming and Using Drones Responsibly**
It is your responsibility to operate UAVs safely and responsibly. This library is intended for educational and research purposes. The developers are not liable for any damages, losses, or injuries incurred while using the software. Always follow local drone laws and guidelines.
