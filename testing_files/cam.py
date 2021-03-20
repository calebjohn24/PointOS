from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (600, 600)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(600, 600))


camera.start_preview()
time.sleep(0.1)



for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    start = time.time()
    img_imm = frame.array
    dst = cv2.cvtColor(img_imm, cv2.COLOR_BGR2RGB)

    dst = dst[350:600, 100:500]
    dst[:,:,2] = 0
    dst[:,:,1] = 0

    hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV) 


    lower_range = np.array([110,70,70])
    upper_range = np.array([130,255,255])

    
    mask = cv2.inRange(hsv, lower_range, upper_range)


    indices = np.where(mask != [0])
    try:

        x_coordinates = indices[1]

        x_coordinates = np.sort(x_coordinates)

        x_coor = np.median(x_coordinates)

        y_coordinates = indices[0]

        y_coordinates = np.sort(y_coordinates)

        y_coor = np.median(y_coordinates)
        print(x_coor, y_coor, 'coordinates')

        mask_out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_out = cv2.circle(
            mask_out, (int(x_coor), int(y_coor)), 10, (0, 0, 255), 2)

    except Exception as e:
        mask_out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        print('e', e)

    cv2.imwrite('raw.jpg', dst)
    cv2.imwrite('mask.jpg', mask_out)


    #print(fps, 'fps')
    print('------')

    rawCapture.truncate(0)