# File DroneVision.py

This file handles real-time video streaming, image processing, and frame buffering for the Bebop2 drone. It provides methods for managing video streams, saving frames, and executing user-defined callbacks for processing frames.

---

## **Classes and Methods**

---

### **SingletonDirectoryManager**

This class ensures that image directories are managed as a singleton, preventing redundant directory creation and optionally cleaning old images.

- **`__new__(cls, path: Path, cleanup_old_images: bool) -> 'SingletonDirectoryManager'`**  
  Ensures only one instance of the directory manager is created and initializes the directory with the specified settings.
  
- **`_initialize(self, path: Path, cleanup_old_images: bool) -> None`**  
  Initializes the directory manager with the provided path and cleanup settings, preparing the directory for use.
  
- **`_prepare_directory(self) -> None`**  
  Creates the directory if it doesn't exist and cleans up old images if required.

---

### **DroneVision**

This class provides the functionality for managing the Bebop2 drone's video stream, buffering frames, and running user-defined frame processing functions.

- **`__init__(self, drone: Bebop2, buffer_size: int = 200, cleanup_old_images: bool = True) -> None`**  
  Initializes the DroneVision instance, setting up the drone's video feed, buffer size, and directory for saving images. It also creates the necessary directory using the SingletonDirectoryManager.
  
- **`set_user_callback(self, callback: Optional[Callable] = None, *args) -> None`**  
  Registers a user-defined callback to process each new frame. The callback is executed in a separate thread.

- **`open_camera(self) -> bool`**  
  Opens the video stream from the drone's camera. Returns `True` if the camera opened successfully, otherwise returns `False`.

- **`start_video_stream(self) -> None`**  
  Starts the video buffering and user callback threads, enabling real-time video streaming.

- **`_execute_user_callback(self, callback: Callable, *args) -> None`**  
  Continuously executes the user-defined callback for each new frame. This method runs in a separate thread.

- **`_buffer_frames(self) -> None`**  
  Buffers frames from the drone's camera into a circular buffer. Each frame is added to the buffer at a rate that matches the expected frame rate.

- **`get_latest_frame(self) -> Optional[cv2.Mat]`**  
  Retrieves the most recent valid frame from the buffer. Returns `None` if no valid frame is available.

- **`close_camera(self) -> None`**  
  Stops video streaming and joins all related threads, cleaning up any ongoing operations.

- **`save_frame_to_disk(self, frame: cv2.Mat, filename: Optional[str] = None) -> None`**  
  Saves a frame to the disk in the designated image directory. Optionally accepts a custom filename; otherwise, a sequential index is used.

--- 

This code structure allows for flexible video management and frame processing, with a focus on real-time performance and user-driven image processing.