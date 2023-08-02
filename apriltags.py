from dt_apriltags import Detector
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import sys
import signal
from pid import PID

def detectTags():
    cameraMatrix = np.array([1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3, 3))
    camera_params = (cameraMatrix[0, 0], cameraMatrix[1, 1], cameraMatrix[0, 2], cameraMatrix[1, 2])
    at_detector = Detector(families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)
    return camera_params, at_detector

def getTag(frame, at_detector, camera_params):
    # video.set(cv2.CAP_PROP_POS_FRAMES, frameNumber)

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(img, True, camera_params, tag_size=0.1)

        # for idx in range(len(tag.corners)):
        #     cv2.line(frame, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
        #     cv2.circle(frame, (x, y), 50, (0, 0, 255), 2)
    return tags

def getTagCenter(tags):
    detected_tags = []
    for tag in tags:
        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
        detected_tags.append([cX, cY])
    return detected_tags

def drawTag(frame):
    tags = getTag(frame)
    height = frame.shape[0]
    width = frame.shape[1]
    centerY = int(height/2)
    centerX = int(width/2)

    for tag in tags:
        (ptA, ptB, ptC, ptD) = tag.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), 3)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 3)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 3)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 3)
        # draw circle in center of tag
        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # draws line between center and circle
        cv2.line(frame, (cX, cY), (centerX, centerY), (255, 0, 0), 3)
        cv2.putText(frame, f"({centerX}, {centerY + 20})", (centerX, centerY), 0, 1, (255, 0, 0), 3)
        cv2.putText(frame, f"({cX}, {cY})", (cX, cY + 20), 0, 1, (255, 0, 0), 3)

        errorPercentY = (centerY - cY)/height * 100
        errorPercentX = (centerX - cX)/width * 100
        vertLabel = (0, int(centerY/2))
        horizLabel = (0, int(centerY/2-30))

        cv2.putText(frame, f'Vertical Distance Percentage: {round(errorPercentY, 3)}%', vertLabel, 0, 1, (255, 0, 255), 3)
        cv2.putText(frame, f'Lateral Distance Percentage: {round(errorPercentX, 3)}%', horizLabel, 0, 1, (255, 0, 255), 3)
    return frame


def tagVideo(vid):
    output_video = cv2.VideoWriter('output_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1920, 1080))
    # video.release() #Save video to disk.
    # total_frames = []
    # Capture frame-by-frame

    count = 1
    ret, frame = vid.read()

    # change comments for set number of frames
    # while ret:
    while True:
        ret, frame = vid.read()
        if ret:
            output_video.write(drawTag(frame))
        print(ret)
        print(f"Frame: {count}")
        count += 1

        # change comments for set number of frames
        if count > 200:
            break

    output_video.release()



def sizeV(video):
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    middle_x = width/2
    middle_y = height/2

    return middle_x, middle_y, width, height

def sizeF(frame):
    width = frame.shape[1]
    height = frame.shape[0]

    middle_x = int(width/2)
    middle_y = int(height/2)

    return middle_x, middle_y, width, height

def main():
    video = 'AprilTagTest.mkv'
    #mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    pid = PID(35, 0.0, 10, 2)
  
    x = 0

    try:
        video = cv2.VideoCapture(video)
        middle_x, middle_y, width, height = sizeV(video)

        while True:
            # Read the current coordinates from the AprilTag detector
            ret, frame = video.read()
            detected_tags = []
            if ret:
                detected_tags = getTagCenter(getTag(frame, at_detector, camera_params))
            else:  
                continue
            # if not detected_tags:
            #     print("No tags found in frame", x)
            #     break

            # For simplicity, assume only one tag is detected in each frame
            if len(detected_tags) != 0:
                tagX, tagY = detected_tags[0]
                print(detected_tags)
                print(middle_x - tagX)
                print(middle_y - tagY)

                # Calculate percent error from the desired middle coordinates
                error_x = round((middle_y - tagY)/height * 100, 6)
                error_y = round((middle_x - tagX)/width * 100, 6)

                print("Frame:", x)
                print(f"Error X: {error_x}%")
                print(f"Error Y: {error_y}%")
            else:
                print("Frame:", x)
                print("Error Y: No tag detected.")
                print("Error X: No tag detected.")

            # Update the PID controllers and get the output
            output_y = pid.update(error_y)
            output_x = pid.update(error_x)

            print("Output X:", output_x)
            print("Output Y:", output_y)

            x += 1
            
    except KeyboardInterrupt:
        print("Interrupted by user.")
        # finally:
        # Stop the vehicle's movement when the program ends
        # set_vertical_power(mav, 0)
        # set_rc_channel_pwm(mav, 6, pwm=1500)


if __name__ == "__main__":
    camera_params, at_detector = detectTags()
    # vida = cv2.VideoCapture('AprilTagTest.mkv')
    # tagVideo(vida)
    main()
    