import cv2
import gi
import numpy as np
from threading import Thread, Event
from time import sleep
from pymavlink import mavutil
from video import Video
from bluerov_interface import BlueROV
import lane_detection
import lane_following
import pid as PID

# Create the video object
video = Video()

# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

pid_horizontal = PID(K_p=20, K_i=0.0, K_d=30, integral_limit=1)
lateral_power = 0


def _get_frame():
    global frame, lateral_power
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        while True:
            #print("HERE")
            if video.frame_available():
                frame = video.frame()
                height, width, _ = frame.shape
                lines = lane_detection.detect_lines(frame)
                print(f"lines:{lines}")
                if lines is not None:
                    lanes = lane_detection.detect_lanes(frame,lines)
                    print(len(lanes))
                    lane_detection.draw_lanes(frame,lanes)
                    center = lane_following.get_lane_center(frame, lanes)
                    print(f"{type(center)}, center: {center}")
                    dist = center[0]-(width/2)
                    percentx = (0.5 - dist / width) * 2
                    lateral_power = pid_horizontal.update(percentx)
                    # lane_result = lane_following.recommend_direction(frame, center[0],center[1], pid_horizontal)
                    # print(lane_result)
                    cv2.imwrite("test_lane.png",frame)
                else:
                    print("No lines detected")
                


    except KeyboardInterrupt:
        return


def _send_rc():
    global vertical_power, lateral_power
    bluerov.set_rc_channels_to_neutral()
    # bluerov.set_rc_channel(9, 1100)
    bluerov.arm()
    # For OLD ROBOT Uncomment below line. For NEW robot, comment it
    # bluerov.mav_connection.set_mode(19)
    while True:
        bluerov.arm()
        bluerov.set_lateral_power(-int(lateral_power))
    


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

except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.set_rc_channels_to_neutral()
    bluerov.disarm()
    print("Exiting...")