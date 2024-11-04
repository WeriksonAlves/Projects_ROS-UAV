class SetConfig:

    def set_video_stream_mode(self, mode='low_latency'):
        """
        Set the video mode for the RTP stream.
        :param: mode: one of 'low_latency', 'high_reliability' or 'high_reliability_low_framerate'

        :return: True if the command was sent and False otherwise
        """

        # handle case issues
        fixed_mode = mode.lower()

        if (fixed_mode not in ("low_latency", "high_reliability", "high_reliability_low_framerate")):
            print("Error: %s is not a valid stream mode.  Must be one of %s" % (mode, "low_latency, high_reliability or high_reliability_low_framerate"))
            print("Ignoring command and returning")
            return False


        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("ardrone3",
                                                                                      "MediaStreaming", "VideoStreamMode", mode)

        return self.drone_connection.send_enum_command_packet_ack(command_tuple,enum_tuple)

    def set_max_altitude(self, altitude):
        """
        Set max altitude in meters.

        :param altitude: altitude in meters
        :return:
        """
        if (altitude < 0.5 or altitude > 150):
            print("Error: %s is not valid altitude. The altitude must be between 0.5 and 150 meters" % altitude)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "PilotingSettings", "MaxAltitude")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[altitude], param_type_tuple=['float'])

        while (not self.sensors.max_altitude_changed):
            self.smart_sleep(0.1)

    def set_max_distance(self, distance):
        """
        Set max distance between the takeoff and the drone in meters.

        :param distance: distance in meters
        :return:
        """
        if (distance < 10 or distance > 2000):
            print("Error: %s is not valid altitude. The distance must be between 10 and 2000 meters" % distance)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "PilotingSettings", "MaxDistance")

        self.sensors.max_distance_changed = False

        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[distance], param_type_tuple=['float'])

        while (not self.sensors.max_distance_changed):
            self.smart_sleep(0.1)

    def enable_geofence(self, value):
        """
	     If geofence is enabled, the drone won't fly over the given max distance.
         1 if the drone can't fly further than max distance, 0 if no limitation on the drone should be done.

        :param value:
        :return:
        """
        if (value not in (0, 1)):
            print("Error: %s is not valid value. Valid value: 1 to enable geofence/ 0 to disable geofence" % value)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "PilotingSettings", "NoFlyOverMaxDistance")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[value], param_type_tuple=['u8'])

        while (not self.sensors.no_fly_over_max_distance_changed):
            self.smart_sleep(0.1)

    def set_max_tilt(self, tilt):
        """
        Set max pitch/roll in degrees

        :param tilt: max tilt for both pitch and roll in degrees
        :return:
        """
        if (tilt < 5 or tilt > 30):
            print("Error: %s is not valid tilt. The tilt must be between 5 and 30 degrees" % tilt)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "PilotingSettings", "MaxTilt")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[tilt], param_type_tuple=['float'])

        while (not self.sensors.max_tilt_changed):
            self.smart_sleep(0.1)

    def set_max_tilt_rotation_speed(self, speed):
        """
        Set max pitch/roll rotation speed in degree/s

        :param speed: max rotation speed for both pitch and roll in degree/s
        :return:
        """
        if (speed < 80 or speed > 300):
            print("Error: %s is not valid speed. The speed must be between 80 and 300 degree/s" % speed)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "SpeedSettings", "MaxPitchRollRotationSpeed")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[speed], param_type_tuple=['float'])

        while (not self.sensors.max_pitch_roll_rotation_speed_changed):
            self.smart_sleep(0.1)

    def set_max_vertical_speed(self, speed):
        """
        Set max vertical speed in m/s

        :param speed: max vertical speed in m/s
        :return:
        """
        if (speed < 0.5 or speed > 2.5):
            print("Error: %s is not valid speed. The speed must be between 0.5 and 2.5 m/s" % speed)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "SpeedSettings", "MaxVerticalSpeed")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[speed], param_type_tuple=['float'])

        while (not self.sensors.max_vertical_speed_changed):
            self.smart_sleep(0.1)

    def set_max_rotation_speed(self, speed):
        """
        Set max yaw rotation speed in degree/s

        :param speed: max rotation speed for yaw in degree/s
        :return:
        """
        if (speed < 10 or speed > 200):
            print("Error: %s is not valid speed. The speed must be between 10 and 200 degree/s" % speed)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "SpeedSettings", "MaxRotationSpeed")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[speed], param_type_tuple=['float'])

        while (not self.sensors.max_rotation_speed_changed):
            self.smart_sleep(0.1)

    def set_hull_protection(self, present):
        """
        Set the presence of hull protection - this is only needed for bebop 1
       	1 if present, 0 if not present

        :param present:
        :return:
        """
        if (present not in (0, 1)):
            print("Error: %s is not valid value. The value must be 0 or 1" % present)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "SpeedSettings", "HullProtection")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[present], param_type_tuple=['u8'])

        while (not self.sensors.hull_protection_changed):
            self.smart_sleep(0.1)

    def set_indoor(self, is_outdoor):
        """
        Set bebop 1 to indoor mode (not used in bebop 2!!)
       	1 if outdoor, 0 if indoor

        :param present:
        :return:
        """
        if (is_outdoor not in (0, 1)):
            print("Error: %s is not valid value. The value must be 0 or 1" % is_outdoor)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "SpeedSettings", "Outdoor")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[is_outdoor], param_type_tuple=['u8'])

        #while (not self.sensors.outdoor_mode_changed):
        #    self.smart_sleep(0.1)

    def set_picture_format(self, format):
        """
        Set picture format

        :param format:
        :return:
        """
        if (format not in ('raw', 'jpeg', 'snapshot', 'jpeg_fisheye')):
            print("Error: %s is not valid value. The value must be : raw, jpeg, snapshot, jpeg_fisheye" % format)
            print("Ignoring command and returning")
            return

        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("ardrone3", "PictureSettings", "PictureFormatSelection", format)
        self.drone_connection.send_enum_command_packet_ack(command_tuple, enum_tuple)

        while (not self.sensors.picture_format_changed):
            self.smart_sleep(0.1)

    def set_white_balance(self, type):
        """
        Set white balance

        :param type:
        :return:
        """
        if (type not in ('auto', 'tungsten', 'daylight', 'cloudy', 'cool_white')):
            print("Error: %s is not valid value. The value must be : auto, tungsten, daylight, cloudy, cool_white" % type)
            print("Ignoring command and returning")
            return

        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("ardrone3", "PictureSettings", "AutoWhiteBalanceSelection", type)
        self.drone_connection.send_enum_command_packet_ack(command_tuple, enum_tuple)

        while (not self.sensors.auto_white_balance_changed):
            self.smart_sleep(0.1)

    def set_exposition(self, value):
        """
        Set image exposure

        :param value:
        :return:
        """
        if (value < -1.5 or value > 1.5):
            print("Error: %s is not valid image exposure. The value must be between -1.5 and 1.5." % value)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "PictureSettings", "ExpositionSelection")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[value], param_type_tuple=['float'])

        while (not self.sensors.exposition_changed):
            self.smart_sleep(0.1)

    def set_saturation(self, value):
        """
        Set image saturation

        :param value:
        :return:
        """
        if (value < -100 or value > 100):
            print("Error: %s is not valid image saturation. The value must be between -100 and 100." % value)
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "PictureSettings", "SaturationSelection")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[value], param_type_tuple=['float'])

        while (not self.sensors.saturation_changed):
            self.smart_sleep(0.1)

    def set_timelapse(self, enable, interval=8):
        """
        Set timelapse mode

        :param enable:
        :param interval:
        :return:
        """
        if (enable not in (0, 1) or interval < 8 or interval > 300):
            print("Error: %s or %s is not valid value." % (enable, interval))
            print("Ignoring command and returning")
            return

        command_tuple = self.command_parser.get_command_tuple("ardrone3", "PictureSettings", "TimelapseSelection")
        self.drone_connection.send_param_command_packet(command_tuple, param_tuple=[enable, interval], param_type_tuple=['u8', 'float'])

        while (not self.sensors.timelapse_changed):
            self.smart_sleep(0.1)

    def set_video_stabilization(self, mode):
        """
        Set video stabilization mode

        :param mode:
        :return:
        """
        if (mode not in ('roll_pitch', 'pitch', 'roll', 'none')):
            print("Error: %s is not valid value. The value must be : roll_pitch, pitch, roll, none" % mode)
            print("Ignoring command and returning")
            return

        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("ardrone3", "PictureSettings", "VideoStabilizationMode", mode)
        self.drone_connection.send_enum_command_packet_ack(command_tuple, enum_tuple)

        while (not self.sensors.video_stabilization_changed):
            self.smart_sleep(0.1)

    def set_video_recording(self, mode):
        """
        Set video recording mode

        :param mode:
        :return:
        """
        if (mode not in ('quality', 'time')):
            print("Error: %s is not valid value. The value must be : quality, time" % mode)
            print("Ignoring command and returning")
            return

        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("ardrone3", "PictureSettings", "VideoRecordingMode", mode)
        self.drone_connection.send_enum_command_packet_ack(command_tuple, enum_tuple)

        while (not self.sensors.video_recording_changed):
            self.smart_sleep(0.1)

    def set_video_framerate(self, framerate):
        """
        Set video framerate

        :param framerate:
        :return:
        """
        if (framerate not in ('24_FPS', '25_FPS', '30_FPS')):
            print("Error: %s is not valid value. The value must be : 24_FPS, 25_FPS, 30_FPS" % framerate)
            print("Ignoring command and returning")
            return

        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("ardrone3", "PictureSettings", "VideoFramerate", framerate)
        self.drone_connection.send_enum_command_packet_ack(command_tuple, enum_tuple)

        while (not self.sensors.video_framerate_changed):
            self.smart_sleep(0.1)

    def set_video_resolutions(self, type):
        """
        Set video resolutions

        :param type:
        :return:
        """
        if (type not in ('rec1080_stream480', 'rec720_stream720')):
            print("Error: %s is not valid value. The value must be : rec1080_stream480, rec720_stream720" % type)
            print("Ignoring command and returning")
            return

        (command_tuple, enum_tuple) = self.command_parser.get_command_tuple_with_enum("ardrone3", "PictureSettings", "VideoResolutions", type)
        self.drone_connection.send_enum_command_packet_ack(command_tuple, enum_tuple)

        while (not self.sensors.video_resolutions_changed):
            self.smart_sleep(0.1)