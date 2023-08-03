from threading import Thread, Event
from time import sleep
from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
from move import process_frame
import numpy as np
import cv2
from dt_apriltags import Detector

# Create the video object
video = Video()

# Create the PID object
pid_vertical = PID(K_p=25, K_i=0.0, K_d=0.01, integral_limit=1) #the robot reaches the april tag and then floats up instead of maintaining its depth
pid_horizontal = PID(K_p=20, K_i=0.0, K_d=30, integral_limit=1)

# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

vertical_power = 0
lateral_power = 0

# Create detector
at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)

camera_matrix = np.array([ 353.571428571, 0, 320, 0, 353.571428571, 180, 0, 0, 1]).reshape((3,3))

camera_params = (
    camera_matrix[0, 0],
    camera_matrix[1, 1],
    camera_matrix[0, 2],
    camera_matrix[1, 2],
)


# find the output of error(error: distance from center to april tag)
def find_pid(tags, width, height):
    finalx = 0
    finaly = 0
    for tag in tags:
        (cX, cY) = int(tag.center[0]), int(tag.center[1])
        finaly = pid_vertical.update(cX)
        finalx = pid_horizontal.update(cY)
    return finalx, finaly


def _get_frame():
    global frame, vertical_power, lateral_power
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        while True:
            # print("HERE")
            if video.frame_available():
                frame = video.frame()
                # print("HERE2")
                height, width, _ = frame.shape  
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                tags = at_detector.detect(frame, True,  camera_params, 0.1)
                # ^ This is the X and Y of every AprilTag in the pic
                if len(tags):
                    for tag in tags:
                        # TODO: change this please
                        print(f"width: {width, tag.center[0]/width}, height: {height, tag.center[1]/height}")
                        percentx = (0.5 - tag.center[0] / width) * 2
                        percenty = (0.5 - tag.center[1] / height) * 2
                        print(f"x percent: {percentx}, y percent: {percenty}")
                        # if tag.center[0] < width / 2:
                        #     percentx = -percentx
                        # if tag.center[1] < height / 2:
                        #     percenty = -percenty
                        outputx = pid_horizontal.update(percentx)
                        outputy = pid_vertical.update(percenty)
                    vertical_power = outputy
                    lateral_power = outputx
                else:
                    print("no tags")
                    vertical_power = 0
                    lateral_power = 0
                print(f"power v: {vertical_power}, h: {lateral_power}")
                


    except KeyboardInterrupt:
        return


def _send_rc():
    global vertical_power, lateral_power
    bluerov.set_rc_channels_to_neutral()
    # bluerov.set_rc_channel(9, 1100)
    bluerov.arm()
    # For OLD ROBOT Uncomment below line. For NEW robot, comment it
    #bluerov.mav_connection.set_mode(19)
    while True:
        bluerov.arm()
        bluerov.set_vertical_power(int(vertical_power))
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
        #_get_frame()

except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.set_rc_channels_to_neutral()
    bluerov.disarm()
    print("Exiting...")