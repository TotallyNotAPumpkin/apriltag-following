from threading import Thread, Event
from time import sleep

from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
from numpy import clip

# TODO: import your processing functions
from apriltags import getTag, getTagCenter, sizeF, detectTags
camera_params, at_detector = detectTags()
# Create the video object
video = Video()
# Create the PID object
pid_vertical = PID(K_p=5, K_i=0.0, K_d=-0.7, integral_limit=1)
pid_horizontal = PID(K_p=5, K_i=0.0, K_d=-0.7, integral_limit=1)
# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

vertical_power = 0
lateral_power = 0


def _get_frame():
    global frame
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        while True:
            if video.frame_available():
                frame = video.frame()
                # TODO: Add frame processing here
                
                middle_x, middle_y, width, height = sizeF(frame)
                detected_tags = getTagCenter(getTag(frame, at_detector, camera_params))
                if len(detected_tags) != 0:
                    tagX, tagY = detected_tags[0]
                    print(detected_tags)

                    # Calculate percent error from the desired middle coordinates
                    error_y = round((middle_y - tagY)/height * 100, 6)
                    error_x = round((middle_x - tagX)/width * 100, 6)

                    print(f"Error X: {error_x}%")
                    print(f"Error Y: {error_y}%")

                    # Update the PID controllers and get the output
                    vertical_power = pid_vertical.update(error_y)
                    lateral_power = pid_horizontal.update(error_x)

                    print("Output X:", lateral_power)
                    print("Output Y:", vertical_power)

                    if vertical_power < -100 or vertical_power > 100:
                        print("Vertical power value out of range. Clipping...")
                        vertical_power = clip(vertical_power, -100, 100)
                        vertical_power = int(vertical_power)

                    if lateral_power < -100 or lateral_power > 100:
                        print("Lateral power value out of range. Clipping...")
                        lateral_power = clip(lateral_power, -100, 100)
                        lateral_power = int(lateral_power)
                else:
                    vertical_power = 0
                    lateral_power = 0
                    print("Error Y: No tag detected.")
                    print("Error X: No tag detected.")



                # TODO: set vertical_power and lateral_power here




                print(frame.shape)
    except KeyboardInterrupt:
        return


def _send_rc():
    while True:
        mav_comn.wait_heartbeat()
        # bluerov.arm()
        bluerov.set_vertical_power(vertical_power)
        bluerov.set_lateral_power(lateral_power)


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
    bluerov.disarm()
    print("Exiting...")
