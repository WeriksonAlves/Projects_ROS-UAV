import cv2
import threading
import time
import os
from typing import Callable, Optional
from pathlib import Path
from py_uav.Bebop2 import Bebop2


class DroneVision:
    """
    Manages real-time video streaming and image processing from a Bebop2 drone.

    Integrates with Bebop2ROS for camera control and provides a callback
    function interface for custom image processing.
    """

    def __init__(self, drone_object: Bebop2,
                 buffer_size: int = 200, cleanup_old_images: bool = True
                 ) -> None:
        """
        Initialize the DroneVision instance.

        :param drone_object: Reference to the Bebop2ROS instance.
        :param buffer_size: Number of frames to buffer in memory.
        :param cleanup_old_images: If True, remove old images from the
                                    directory.
        """
        self.drone_object = drone_object
        self.buffer_size = buffer_size
        self.cleanup_old_images = cleanup_old_images
        self.image_index = 1

        # Path configuration for images
        main = Path(os.path.dirname(__file__))
        self.image_dir = main / "images" / "internal"
        self._prepare_image_directory()

        # Video buffer for frame storage
        self.buffer = [None] * buffer_size
        self.buffer_index = 0
        self.vision_running = True
        self.new_frame = False

        # Thread management for video and callback
        self.vision_thread = threading.Thread(target=self._buffer_vision)
        self.user_vision_thread = None

    def set_user_callback(self, callback_function: Optional[Callable] = None,
                          *callback_args) -> None:
        """
        Set an optional callback for processing vision frames.

        :param callback_function: Callback function for custom frame handling.
        :param callback_args: Additional arguments for the callback function.
        """
        if callback_function:
            self.user_vision_thread = threading.Thread(
                target=self._user_callback,
                args=(callback_function, *callback_args))

    def open_camera(self) -> bool:
        """
        Opens the video stream using Bebop2ROS camera controls.

        :return: True if the stream opened successfully, False otherwise.
        """
        success = self.drone_object.sensor_manager.check_camera()
        if success:
            self._start_video_buffering()
        return success

    def _prepare_image_directory(self) -> None:
        """
        Prepares the image directory by cleaning up old images if requested.
        """
        if self.cleanup_old_images and self.image_dir.exists():
            for image_file in self.image_dir.glob("*.png"):
                image_file.unlink()
        self.image_dir.mkdir(exist_ok=True)

    def _start_video_buffering(self) -> None:
        """
        Starts threads for buffering the video and processing frames.
        """
        print("Starting vision thread.")
        self.vision_thread.start()

        if self.user_vision_thread:
            self.user_vision_thread.start()

    def _user_callback(self, callback_function: Callable, *callback_args
                       ) -> None:
        """
        Calls the user-defined callback with each new frame.

        :param callback_function: Function to process each frame.
        :param callback_args: Additional arguments for the callback.
        """
        while self.vision_running:
            if self.new_frame:
                callback_function(*callback_args)
                self.new_frame = False
            time.sleep(1.0 / (3.0 * 30))  # Faster than FPS to stay in sync

    def _buffer_vision(self) -> None:
        """
        Buffers frames from the video stream, storing each in the buffer.
        """
        while self.vision_running:
            img = self.drone_object.sensor_manager.drone_camera.image_data['compressed']
            if img is not None:
                self.image_index += 1
                self.buffer[self.buffer_index] = img
                self.buffer_index = (
                    self.buffer_index + 1) % self.buffer_size
                self.new_frame = True
            else:
                time.sleep(0.01)  # Short pause to avoid busy-waiting

            # Maintain the buffer at ~2x FPS rate
            time.sleep(1.0 / (2.0 * 30))

    def get_latest_frame(self) -> Optional[cv2.Mat]:
        """
        Retrieves the most recent valid frame from the buffer.

        :return: Latest frame if available, else None.
        """
        return self.buffer[self.buffer_index]

    def close_video_stream(self) -> None:
        """
        Stops the video stream and all related threads.
        """
        self.vision_running = False
        print("Closing video stream.")
        # self.drone_object.stop_video_stream()

        # # Terminate any active video processing
        # if hasattr(self, 'ffmpeg_process'):
        #     self.ffmpeg_process.kill()
        #     self.ffmpeg_process.terminate()
        time.sleep(1)

        if self.vision_thread.is_alive():
            self.vision_thread.join()

        if self.user_vision_thread and self.user_vision_thread.is_alive():
            self.user_vision_thread.join()
