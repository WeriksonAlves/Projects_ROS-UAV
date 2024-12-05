#!/usr/bin/env python3
from rospy_uav.rospy_uav import DroneVision, Bebop2
import cv2


class UserVision:
    """
    Handles real-time image display and saving from the drone's camera.
    """

    def __init__(self, vision: DroneVision) -> None:
        """
        Initializes the UserVision instance.

        :param vision: DroneVision instance responsible for streaming video.
        """
        self.vision = vision
        self.image_index = 0

    def save_image(self) -> None:
        """
        Saves the latest valid frame to disk.
        """
        image = self.vision.get_latest_frame()
        if image is not None:
            main_name = "Projects_ROS-UAV/py_uav/images/image_"
            filename = f"{main_name}{self.image_index:04d}.png"
            cv2.imwrite(filename, image)
            self.image_index += 1

    def display_image(self) -> None:
        """
        Displays the latest valid frame in a window.
        """
        image = self.vision.get_latest_frame()
        if image is not None:
            cv2.imshow("Drone Camera Feed", image)
            cv2.waitKey(1)


class demoBebop2CameraOn:
    """
    Manages the Bebop2 drone's vision system, including video streaming and
    camera controls.
    """

    def __init__(self, drone_type: str = 'bebop2',
                 ip_address: str = '192.168.0.202') -> None:
        """
        Initializes the Bebop2VisionManager instance.

        :param drone_type: Type of the drone (e.g., 'gazebo', 'bebop2').
        """
        self.drone = Bebop2(drone_type=drone_type, ip_address=ip_address)
        self.vision = None
        self.user_vision = None

    def connect_to_drone(self) -> bool:
        """
        Attempts to connect to the Bebop2 drone.

        :return: True if connection is successful, otherwise False.
        """
        print("Connecting to Bebop2 drone...")
        if self.drone.check_connection():
            print("Connection successful.")
            return True
        print("Failed to connect. Please check the connection.")
        return False

    def start_video_stream(self, buffer_size: int = 50,
                           cleanup_old_images: bool = True) -> None:
        """
        Starts the drone's video streaming.

        :param buffer_size: Number of frames to buffer in memory.
        :param cleanup_old_images: Cleans up old images if True.
        """
        print("Starting video stream...")
        self.vision = DroneVision(
            self.drone,
            buffer_size=buffer_size,
            cleanup_old_images=cleanup_old_images
        )
        self.user_vision = UserVision(self.vision)
        self.vision.set_user_callback(self.user_vision.display_image)

        if self.vision.open_camera():
            print("Video stream started successfully.")
        else:
            print("Video stream failed to start.")

    def stop_video_stream(self) -> None:
        """
        Stops the video streaming from the drone.
        """
        print("Stopping video stream...")
        if self.vision:
            self.vision.close_camera()

    def adjust_camera(self, tilt: int, pan: int) -> None:
        """
        Adjusts the camera tilt and pan.

        :param tilt: Camera tilt angle (in degrees).
        :param pan: Camera pan angle (in degrees).
        """
        print(f"Adjusting camera to tilt={tilt}°, pan={pan}°")
        self.drone.adjust_camera_orientation(tilt=tilt, pan=pan)
        self.drone.smart_sleep(2)

    def run_experiment(self) -> None:
        """
        Runs the vision experiment sequence.
        """
        if self.drone.drone_type == 'bebop2':
            if not self.connect_to_drone():
                return

        self.start_video_stream()

        self.adjust_camera(tilt=0, pan=0)
        self.drone.smart_sleep(5)
        self.adjust_camera(tilt=45, pan=45)
        self.drone.smart_sleep(5)

        print("Experiment complete.")
        self.stop_video_stream()


if __name__ == "__main__":
    vision_manager = demoBebop2CameraOn(drone_type='gazebo')
    vision_manager.run_experiment()
