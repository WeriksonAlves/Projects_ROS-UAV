from py_uav.Bebop2 import Bebop2

# Assuming Bebop2ROS is already defined and imported in your project
bebop_drone = Bebop2(drone_type='Gazebo')

print("Connecting to Bebop2 drone...")
if True:  # bebop_drone.check_connection():
    print("Successfully connected to the drone.")

    print(bebop_drone.sensor_manager.sensor_data['battery_level'])

    # Start the video stream
    print("Starting video stream...")
    bebop_drone.start_video_stream()
    bebop_drone.smart_sleep(2)  # Wait for the stream to stabilize

    # Update drone's state to retrieve the latest sensor data
    print("Requesting state update...")
    bebop_drone.update_sensors()

    bebop_drone.takeoff()

    # # set safe indoor parameters
    # bebop_drone.state_manager.set_max_tilt(5)
    # bebop_drone.state_manager.set_max_vertical_speed(1)

    print("Flying direct: Slow move for indoors")
    bebop_drone.fly_direct(0, 0, 10, 0, duration=5)
    bebop_drone.fly_direct(10, 0, 0, 0, duration=5)
    bebop_drone.fly_direct(0, 10, 0, 0, duration=5)
    bebop_drone.fly_direct(0, 0, 0, 0, duration=10)

    bebop_drone.smart_sleep(2)

    bebop_drone.land()
    if bebop_drone.is_landed():
        bebop_drone.emergency_land()

    print("DONE - Ending experiment.")
    print(bebop_drone.sensor_manager.sensor_data['battery_level'])

else:
    print(
        "Failed to connect to the Bebop2 drone. Please check the connection"
        "and try again.")
