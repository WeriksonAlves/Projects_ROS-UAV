import cv2
import threading
import time
import os
from typing import Callable, Optional
from pathlib import Path
from py_uav.Bebop2 import Bebop2


class SingletonDirectoryManager:
    """Singleton pattern to manage the creation of image directories."""

    _instance = None

    def __new__(cls, path: Path, cleanup_old_images: bool
                ) -> 'SingletonDirectoryManager':
        if not cls._instance:
            cls._instance = super(SingletonDirectoryManager, cls).__new__(cls)
            cls._instance.path = path
            cls._instance.cleanup_old_images = cleanup_old_images
            cls._instance._prepare_directory()
        return cls._instance

    def _prepare_directory(self) -> None:
        """Prepare the directory by cleaning up old images if necessary."""
        if self.cleanup_old_images and self.path.exists():
            for image_file in self.path.glob("*.png"):
                image_file.unlink()
        self.path.mkdir(parents=True, exist_ok=True)


class DroneVision:
    """
    Manages real-time video streaming and image processing from a Bebop2 drone.

    Integrates with Bebop2ROS for camera control and provides a callback
    function interface for custom image processing.
    """

    def __init__(self, drone_object: Bebop2, buffer_size: int = 200,
                 cleanup_old_images: bool = True) -> None:
        """
        Initialize the DroneVision instance.

        :param drone_object: Reference to the Bebop2ROS instance.
        :param buffer_size: Number of frames to buffer in memory.
        :param cleanup_old_images: If True, removes old images from the
                                   directory.
        """
        self.drone_object = drone_object
        self.buffer_size = buffer_size
        self.image_index = 1

        # Image directory setup
        main_path = Path(__file__).resolve().parent
        self.image_dir = main_path / "images" / "internal"
        self.directory_manager = SingletonDirectoryManager(self.image_dir,
                                                           cleanup_old_images)

        # Initialize video buffer
        self.buffer = [None] * buffer_size
        self.buffer_index = 0
        self.vision_running = False
        self.new_frame = threading.Event()

        # Thread management for video streaming and user callback
        self.vision_thread = threading.Thread(target=self._buffer_vision,
                                              daemon=True)
        self.user_vision_thread = None

    def set_user_callback(self, callback_function: Optional[Callable] = None,
                          *callback_args) -> None:
        """
        Sets an optional callback for processing vision frames.

        :param callback_function: Callback function for custom frame handling.
        :param callback_args: Additional arguments for the callback function.
        """
        if callback_function:
            self.user_vision_thread = threading.Thread(
                target=self._user_callback, args=(callback_function,
                                                  *callback_args), daemon=True
            )

    def open_camera(self) -> bool:
        """
        Opens the video stream using Bebop2ROS camera controls.

        :return: True if the stream opened successfully, False otherwise.
        """
        success = self.drone_object.sensor_manager.check_camera()
        if success:
            self.start_video_buffering()
        return success

    def start_video_buffering(self) -> None:
        """
        Starts threads for buffering the video and processing frames.
        """
        print("Starting vision thread.")
        self.vision_running = True
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
            if self.new_frame.is_set():
                callback_function(*callback_args)
                self.new_frame.clear()

            # Faster than FPS to stay in sync without busy-waiting
            time.sleep(1 / 90)

    def _buffer_vision(self) -> None:
        """
        Buffers frames from the video stream, storing each in the buffer.
        """
        while self.vision_running:
            img = self.drone_object.sensor_manager.drone_camera.image_data.get(
                'compressed')
            if img is not None and img.size > 0:  # Check if img is non-empty
                self.image_index += 1
                self.buffer[self.buffer_index] = img
                self.buffer_index = (self.buffer_index + 1) % self.buffer_size
                self.new_frame.set()

            # Maintain buffer rate at ~2x FPS for efficiency
            time.sleep(1 / 60)

    def get_latest_frame(self) -> Optional[cv2.Mat]:
        """
        Retrieves the most recent valid frame from the buffer.

        :return: Latest frame if available, else None.
        """
        # Check if the buffer at the latest index is not None and contains data
        latest_frame = self.buffer[self.buffer_index - 1]
        if latest_frame is not None and latest_frame.size > 0:
            return latest_frame
        return None

    def close_video_stream(self) -> None:
        """
        Stops the video stream and all related threads.
        """
        self.vision_running = False
        print("Closing video stream.")

        if self.vision_thread.is_alive():
            self.vision_thread.join()

        if self.user_vision_thread and self.user_vision_thread.is_alive():
            self.user_vision_thread.join()
