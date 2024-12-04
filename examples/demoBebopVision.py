from ..rospy_uav import DroneVision, Bebop2
import cv2
import rospy


class UserVision:
    """
    A class responsible for saving and displaying images captured from the
    drone's camera in real time.
    """

    def __init__(self, vision: DroneVision) -> None:
        """
        Initializes the UserVision instance.

        :param vision: The DroneVision instance responsible for streaming
                        video from the drone.
        """
        self.index = 0
        self.vision = vision

    def save_picture(self) -> None:
        """
        Saves the last valid image captured by the drone's camera to a file.
        """
        img = self.vision.get_latest_frame()
        if img is not None:
            main_name = "Projects_ROS-UAV/py_uav/images/image_"
            filename = f"{main_name}{self.index:04d}.png"
            cv2.imwrite(filename, img)
            self.index += 1

    def display_picture(self) -> None:
        """
        Displays the last valid image captured by the drone's camera in a
        window.
        """
        img = self.vision.get_latest_frame()
        if img is not None:
            cv2.imshow("Drone Camera", img)
            cv2.waitKey(1)


class Bebop2Vision:
    """
    A class to control the Bebop2 drone and handle its operations, including
    connection, video streaming, and camera adjustments.
    """

    def __init__(self, drone_type: str = 'bebop2') -> None:
        """
        Initializes the DroneController with a specified drone type.

        :param drone_type: Type of the drone (e.g., 'Gazebo' or 'bebop2').
        """
        self.bebop = Bebop2(drone_type=drone_type, ip_address='192.168.0.202')
        self.vision = None
        self.user_vision = None

    def connect_to_drone(self) -> bool:
        """
        Connects to the Bebop2 drone.

        :return: True if the drone was successfully connected, False otherwise.
        """
        try:
            if self.bebop.check_connection():
                print("Bebop2 connected.")
                return True
            print("Error connecting to Bebop. Check the connection.")
            return False
        except KeyboardInterrupt:
            print("Shutdown requested. Cleaning up...")
        finally:
            self.bebop._shutdown_flag = True  # Signal threads to stop
            rospy.signal_shutdown("User requested shutdown")
            print("Clean shutdown complete.")

    def print_battery_level(self) -> None:
        """
        Prints the current battery level of the drone.
        """
        battery_level = self.bebop.sensor_manager.sensor_data.get(
            'battery_level', 'Unknown'
        )
        print(f"Battery Level: {battery_level}")

    def start_video_stream(self, buffer_size: int = 50,
                           cleanup_old_images: bool = True) -> None:
        """
        Initializes and starts the video stream from the Bebop2 drone.

        :param buffer_size: The number of frames to buffer in memory.
        :param cleanup_old_images: Whether to clean up old images in the
                                    directory before starting the stream.
        """
        self.vision = DroneVision(self.bebop, buffer_size=buffer_size,
                                  cleanup_old_images=cleanup_old_images)
        self.user_vision = UserVision(self.vision)

        self.vision.set_user_callback(self.user_vision.display_picture)

        if self.vision.open_camera():
            print("Video stream started successfully.")
        else:
            print("Failed to start video stream.")

    def adjust_camera(self, tilt: int, pan: int) -> None:
        """
        Adjusts the camera tilt and pan to specified angles.

        :param tilt: The tilt angle in degrees.
        :param pan: The pan angle in degrees.
        """
        print(f"Adjusting camera to tilt: {tilt}°, pan: {pan}°")
        self.bebop.adjust_camera_orientation(tilt=tilt, pan=pan)
        self.bebop.smart_sleep(2)

    def stop_video_stream(self) -> None:
        """
        Stops the video stream from the Bebop2 drone.
        """
        print("Stopping video stream.")
        self.vision.close_camera()

    def run_experiment(self):
        """
        Executes the full experimental procedure with the Bebop2 drone.
        """
        if not self.connect_to_drone():
            return

        self.print_battery_level()
        self.start_video_stream()

        print("Move the drone manually to capture different images.")
        self.bebop.smart_sleep(5)

        self.adjust_camera(tilt=0, pan=0)
        self.bebop.smart_sleep(5)

        self.adjust_camera(tilt=45, pan=45)
        self.bebop.smart_sleep(5)

        print("Experiment complete.")
        self.stop_video_stream()

        self.print_battery_level()


if __name__ == "__main__":
    drone_vision = Bebop2Vision()
    drone_vision.run_experiment()
