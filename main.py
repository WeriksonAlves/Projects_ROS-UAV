#!/usr/bin/env python
from py_uav.Bebop2 import (
    Bebop2
)
import cv2
import rospy
import time


# Main function
def main():
    rospy.init_node('External_Flight', anonymous=True)

    # Create a new UAV object
    uav = Bebop2()

    start_time = time.time()
    while True:
        if time.time() - start_time > .5:
            start_time = time.time()

            # Exit the loop if the user presses 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Get the current frame from the camera
            success, frame = uav.capture_frame()

            if success:
                # Display the frame
                cv2.imshow("Frame", frame)
            else:
                print("Failed to capture frame.")


main()
