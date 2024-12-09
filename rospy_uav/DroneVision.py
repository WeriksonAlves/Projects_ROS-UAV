from typing import Callable, Optional
from pathlib import Path
from .Bebop2 import Bebop2
import cv2
import threading
import time


class SingletonDirectoryManager:
    """
    Singleton to manage image directories for DroneVision.
    Ensures directory creation and optional cleanup of old images.
    """

    _instance = None

    def __new__(cls, path: Path, cleanup_old_images: bool
                ) -> 'SingletonDirectoryManager':
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialize(path, cleanup_old_images)
        return cls._instance

    def _initialize(self, path: Path, cleanup_old_images: bool) -> None:
        self.path = path
        self.cleanup_old_images = cleanup_old_images
        self._prepare_directory()

    def _prepare_directory(self) -> None:
        """Create or clean the directory based on the configuration."""
        print(f"Preparing image directory: {self.path}")
        print(f"Size of the directory: {len(list(self.path.glob('*.png')))}")
        if self.cleanup_old_images and self.path.exists():
            for image_file in self.path.glob("*.png"):
                image_file.unlink()
        self.path.mkdir(parents=True, exist_ok=True)


class DroneVision:
    """
    Handles real-time video streaming and image processing for the Bebop2
    drone. Provides video buffering and an interface for user-defined frame
    processing.
    """

    def __init__(self, drone: Bebop2, buffer_size: int = 200,
                 cleanup_old_images: bool = True) -> None:
        """
        Initialize the DroneVision instance.

        :param drone: Bebop2 drone instance for accessing camera and sensors.
        :param buffer_size: Number of frames to buffer in memory.
        :param cleanup_old_images: If True, removes old images from the
                                    directory.
        """
        self.drone = drone
        self.buffer_size = buffer_size
        self.image_index = 0

        # Set up image directory using SingletonDirectoryManager
        main_path = Path(__file__).resolve().parent
        self.image_dir = main_path / "images"
        SingletonDirectoryManager(self.image_dir, cleanup_old_images)

        # Initialize video buffer and threading
        self.buffer = [None] * buffer_size
        self.buffer_index = 0
        self.vision_running = False
        self.new_frame_event = threading.Event()

        # Thread management
        self.vision_thread = threading.Thread(target=self._buffer_frames,
                                              daemon=True)
        self.user_callback_thread: Optional[threading.Thread] = None

    def set_user_callback(self, callback: Optional[Callable] = None, *args
                          ) -> None:
        """
        Register a user-defined callback to process each new frame.

        :param callback: Function to process frames.
        :param args: Additional arguments for the callback.
        """
        if callback:
            self.user_callback_thread = threading.Thread(
                target=self._execute_user_callback,
                args=(callback, *args), daemon=True
            )

    def open_camera(self) -> bool:
        """
        Open the video stream using the drone's camera.

        :return: True if the camera opened successfully, otherwise False.
        """
        if self.drone.camera_on():
            self.start_video_stream()
            return True
        return False

    def start_video_stream(self) -> None:
        """
        Start video buffering and user callback threads.
        """
        if not self.vision_running:
            self.vision_running = True
            self.vision_thread.start()
            if self.user_callback_thread:
                self.user_callback_thread.start()

    def _execute_user_callback(self, callback: Callable, *args) -> None:
        """
        Continuously execute the user-defined callback for each new frame.

        :param callback: The user-defined processing function.
        :param args: Additional arguments for the callback.
        """
        while self.vision_running:
            if self.new_frame_event.is_set():
                callback(*args)
                self.new_frame_event.clear()
            time.sleep(1 / 90)  # Slightly faster than expected frame rate

    def _buffer_frames(self) -> None:
        """
        Buffer frames from the drone's camera into a circular buffer.
        """
        while self.vision_running:
            frame = self.drone.sensor_manager.drone_camera.image_data.get(
                'compressed'
            )
            if frame is not None and frame.size > 0:
                self.buffer[self.buffer_index] = frame
                self.buffer_index = (self.buffer_index + 1) % self.buffer_size
                self.image_index += 1
                self.new_frame_event.set()
            time.sleep(1 / 60)  # Match ~2x typical FPS for efficiency

    def get_latest_frame(self) -> Optional[cv2.Mat]:
        """
        Retrieve the most recent valid frame from the buffer.

        :return: The latest frame, or None if no valid frame is available.
        """
        latest_frame = self.buffer[(self.buffer_index - 1) % self.buffer_size]
        if latest_frame is not None and latest_frame.size > 0:
            return latest_frame
        return None

    def close_camera(self) -> None:
        """
        Stop video streaming and join all related threads.
        """
        self.vision_running = False
        if self.vision_thread.is_alive():
            self.vision_thread.join()
        if self.user_callback_thread and self.user_callback_thread.is_alive():
            self.user_callback_thread.join()

    def save_frame_to_disk(self, frame: cv2.Mat,
                           filename: Optional[str] = None) -> None:
        """
        Save a frame to disk in the image directory.

        :param frame: Frame to save.
        :param filename: Optional filename. If None, use a sequential index.
        """
        filename = filename or f"frame_{self.image_index:05d}.png"
        filepath = self.image_dir / filename
        cv2.imwrite(str(filepath), frame)
