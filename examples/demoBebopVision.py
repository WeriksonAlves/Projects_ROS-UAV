"""
Purpose: Demo of the Bebop2 drone vision using the py_uav library.

This script demonstrates how to capture images from the drone's camera in real
time and save them to the local file system.

The script connects to the drone, starts the video stream, and captures images
from the drone's camera. The images are saved to the local file system.

The script also demonstrates how to adjust the camera's pan and tilt angles
using the pan_tilt_camera method.

To run the script, the Bebop2 drone must be connected to the computer via
Wi-Fi. The drone's IP address must be set to
"""
import time
import cv2
from py_uav import (
    DroneVision,
    Bebop2
)


class UserVision:
    """
    Class responsible for saving the images captured from the video in real
    time.
    """
    def __init__(self, vision: DroneVision):
        self.index = 0
        self.vision = vision

    def save_pictures(self):
        """
        Saves the last valid image from the drone's camera.
        """
        img = self.vision.get_latest_frame()
        if img is not None:
            filename = f"Projects_ROS-UAV/py_uav/images/image_{self.index:04d}.png"
            cv2.imwrite(filename, img)
            self.index += 1

    def imshow_pictures(self):
        """
        Displays the last valid image from the drone's camera.
        """
        img = self.vision.get_latest_frame()
        if img is not None:
            cv2.imshow("Drone Camera", img)
            cv2.waitKey(1)


# Initializes the Bebop2ROS object for drone control
bebop = Bebop2()

# Battery level
print(f"Battery level: {bebop.sensor_manager.sensor_data['battery_level']}%")

# Connect to drone
if bebop.check_connection():
    print("Bebop2 connected.")

    # Initialize vision using DroneVision
    bebop_vision = DroneVision(bebop, buffer_size=50, cleanup_old_images=True)
    user_vision = UserVision(bebop_vision)

    # Define the callback function for saving images
    bebop_vision.set_user_callback(user_vision.imshow_pictures)

    # Start video streaming
    if bebop_vision.open_camera():
        print("Video stream started successfully.")

        # Wait to capture images while the drone is static
        print("Move the drone manually to capture different images.")
        time.sleep(5)

        # Simulates camera movement without flying (pan and tilt adjustment)
        print("Adjusting the camera.")
        bebop.pan_tilt_camera(tilt=0, pan=0)
        time.sleep(5)

        bebop.pan_tilt_camera(tilt=45, pan=45)
        time.sleep(5)

        print("Ending the demo and stopping the vision.")
        bebop_vision.close_video_stream()

else:
    print("Error connecting to Bebop. Try again.")
