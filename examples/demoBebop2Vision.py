#!/usr/bin/env python3

from rospy_uav.rospy_uav import DroneVision, Bebop2
import cv2


class ImageHandler:
    """
    Handles saving and displaying images from the drone's video feed.
    """

    def __init__(self, vision: DroneVision,
                 save_path: str = "rospy_uav/rospy_uav/images/") -> None:
        """
        Initializes the ImageHandler.

        :param vision: DroneVision instance for streaming video.
        :param save_path: Directory path to save captured images.
        """
        self.vision = vision
        self.image_index = 0
        self.save_path = save_path

    def save_image(self) -> None:
        """
        Saves the latest valid frame to the specified directory.
        """
        image = self.vision.get_latest_frame()
        if image is not None:
            filename = f"{self.save_path}image_{self.image_index:04d}.png"
            cv2.imwrite(filename, image)
            print(f"Saved image: {filename}")
            self.image_index += 1

    def display_image(self) -> None:
        """
        Displays the latest valid frame in a window.
        """
        image = self.vision.get_latest_frame()
        if image is not None:
            cv2.imshow("Drone Camera Feed", image)
            cv2.waitKey(1)


class DroneCameraManager:
    """
    Manages the drone's video feed and camera operations.
    """

    def __init__(self, drone: Bebop2) -> None:
        """
        Initializes the DroneCameraManager.

        :param drone: Instance of the Bebop2 drone.
        """
        self.drone = drone
        self.vision = None
        self.image_handler = None

    def initialize_camera(self, vision: DroneVision,
                          image_handler: ImageHandler) -> None:
        """
        Sets up the video streaming and initializes the ImageHandler.
        """
        print("Initializing video stream...")
        self.vision = vision
        self.image_handler = image_handler

        if self.vision.open_camera():
            print("Video stream started successfully.")
        else:
            print("Failed to start video stream.")

    def adjust_camera(self, tilt: int, pan: int, duration: int = 5) -> None:
        """
        Adjusts the camera's orientation.

        :param tilt: Tilt angle for the camera.
        :param pan: Pan angle for the camera.
        :param duration: Time to wait after adjustment (in seconds).
        """
        print(f"Adjusting camera: Tilt={tilt}, Pan={pan}")
        self.drone.adjust_camera_orientation(tilt=tilt, pan=pan)
        self.drone.smart_sleep(duration)

    def stop_camera(self) -> None:
        """
        Stops the video stream and closes the camera.
        """
        print("Stopping the video stream...")
        if self.vision:
            self.vision.close_camera()


def main() -> None:
    """
    Main function to execute the drone camera operations.
    """
    # Configuration
    drone_type = 'gazebo'  # Change to 'bebop2' for real drone usage
    ip_address = '192.168.0.202'  # Replace with the real drone's IP address

    # Initialize the drone
    print("Connecting to the drone...")
    bebop = Bebop2(drone_type=drone_type, ip_address=ip_address)
    camera_manager = DroneCameraManager(bebop)

    # Connect to the drone
    if drone_type == 'bebop2' and not bebop.check_connection():
        print("Failed to connect to the drone. Please check the connection.")
        return
    print("Drone connected successfully.")

    # Display battery status
    battery_level = bebop.sensor_manager.sensor_data.get("battery_level",
                                                         "Unknown")
    print(f"Battery Level: {battery_level}%")

    # Start video streaming
    vision = DroneVision(bebop, buffer_size=50, cleanup_old_images=True)
    image_handler = ImageHandler(vision)
    vision.set_user_callback(image_handler.save_image)

    camera_manager.initialize_camera(vision, image_handler)

    print("Move the drone with the handset to capture different images.")

    # Camera adjustments
    camera_manager.adjust_camera(tilt=0, pan=0, duration=2)
    camera_manager.adjust_camera(tilt=45, pan=45, duration=2)

    # End the experiment
    camera_manager.stop_camera()

    print("Experiment complete.")


if __name__ == "__main__":
    main()
