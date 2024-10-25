"""
Purpose:
"""


class Bebop2Indoors:
    """
    """

    def __init__(self):
        """
        Initializes the Bebop2SettingsIndoors class and sets up ROS publishers
        to send commands to the Bebop2.
        """
        pass

    def set_max_altitude(self, altitude: int):
        """
        Set the maximum allowable altitude in meters. The altitude must be
        between 0.5 and 150 meters.
        """
        pass

    def set_max_distance(self, distance: int):
        """
        Set max distance between the takeoff and the drone in meters. The
        distance must be between 10 and 2000 meters.
        """
        pass

    def set_max_tilt(self, tilt: int):
        """
        Set the maximum allowable tilt in degrees for the drone (this limits
        speed). The tilt must be between 5 (very slow) and 30 (very fast)
        degrees.
        """
        pass

    def set_max_tilt_rotation_speed(self, speed: int):
        """
        Set the maximum allowable tilt rotation speed in degree/s. The tilt
        rotation speed must be between 80 and 300 degree/s.
        """
        pass

    def set_max_vertical_speed(self, speed: float):
        """
        Set the maximum allowable vertical speed in m/s. The vertical speed
        must be between 0.5 and 2.5 m/s.
        """
        pass

    def set_max_rotation_speed(self, speed: int):
        """
        Set the maximum allowable rotation speed in degree/s. The rotation
        speed must be between 10 and 200 degree/s.
        """
        pass

    def set_flat_trim(duration: float = 0):
        """
        Tell the Bebop to run with a flat trim. If duration > 0, waits for the
        comand to be acknowledged.
        """
        pass

    def enable_geofence(self, value: int):
        """
        If geofence is enabled, the drone won't fly over the given max
        distance. Valid value: 1 if the drone can't fly further than max
        distance, 0 if no limitation on the drone should be done.
        """
        pass
