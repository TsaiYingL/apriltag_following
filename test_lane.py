import cv2
import gi
import numpy as np
from threading import Thread, Event
from time import sleep
from pymavlink import mavutil
from video import Video
from bluerov_interface import BlueROV
import lane_detection

# Create the video object
video = Video()

# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

def _get_frame():
    global frame
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        while True:
            #print("HERE")
            if video.frame_available():
                frame = video.frame()
                lines = lane_detection.detect_lines(frame)
                if lines is not None:
                    lanes = lane_detection.detect_lanes(frame,lines)
                    print(len(lanes))
                    lane_detection.draw_lanes(frame,lanes)
                    cv2.imwrite("something.png",frame)
                else:
                    print("No lines detected")
                


    except KeyboardInterrupt:
        return


def _send_rc():
    global vertical_power, lateral_power
    bluerov.set_rc_channels_to_neutral()
    # bluerov.set_rc_channel(9, 1100)
    while True:
        bluerov.arm()
    


# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
        #_get_frame()

except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.set_rc_channels_to_neutral()
    bluerov.disarm()
    print("Exiting...")