import numpy as np
from typing import Tuple


class MyFunctions:

    def ajust_camera(self, frame: np.ndarray, bounding_box: np.ndarray,
                     Gi: Tuple[int, int] = (0.5, 0.5), Ge: Tuple[int, int] = (
                         50, 50)) -> None:
        """
        Adjust the camera orientation based on the operator's position in the
        frame.

        :param frame: The captured frame.
        :param boxes: Bounding boxes for detected objects.
        :param Gi: Internal gain for pitch and yaw adjustment.
        :param Ge: External gain for pitch and yaw adjustment.
        """
        dist_center_h, dist_center_v = self.centralize_operator(
            frame, bounding_box)
        sc_pitch = np.tanh(-dist_center_v * Gi[0]) * Ge[0] if abs(
            dist_center_v) > 0.25 else 0
        sc_yaw = np.tanh(dist_center_h * Gi[1]) * Ge[1] if abs(
            dist_center_h) > 0.25 else 0
        print(f"pitch: {sc_pitch}, yaw: {sc_yaw}")
        self.camera.pan_tilt_camera(tilt=sc_pitch, pan=sc_yaw)

    def centralize_operator(self, frame: np.ndarray, bounding_box: Tuple[
        int, int, int, int], drone_pitch: float = 0.0, drone_yaw: float = 0.0
    ) -> Tuple[float, float]:
        """
        Adjust the camera's orientation to center the operator in the frame,
        compensating for yaw and pitch.

        :param frame: The captured frame.
        :param bounding_box: The bounding box of the operator as (x, y, width,
        height).
        :param drone_pitch: The pitch value for compensation.
        :param drone_yaw: The yaw value for compensation.
        :return: Tuple[float, float]: The horizontal and vertical distance to
        the frame's center.
        """
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)

        box_x, box_y, _, _ = bounding_box
        dist_to_center_h = (
            box_x - frame_center[0]) / frame_center[0] - drone_yaw
        dist_to_center_v = (
            box_y - frame_center[1]) / frame_center[1] - drone_pitch

        return dist_to_center_h, dist_to_center_v
