from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (400, 400)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(400, 400))


camera.start_preview()
time.sleep(0.1)



for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    start = time.time()
    img_imm = frame.array
    dst = cv2.cvtColor(img_imm, cv2.COLOR_BGR2RGB)

    dst = dst[200:400, 0:400]

    #dst[:,:,1] = 0
    #dst[:,:,0] = 0

    lower_black = np.array([0, 0, 0], dtype="uint16")
    upper_black = np.array([0, 0, 90], dtype="uint16")
    black_mask = cv2.inRange(dst, lower_black, upper_black)
    

    indices = np.where(black_mask != [0])
    try:

        x_coordinates = indices[1]

        x_coordinates = np.sort(x_coordinates)

        x_coor = np.median(x_coordinates)

        y_coordinates = indices[0]

        y_coordinates = np.sort(y_coordinates)

        y_coor = np.median(y_coordinates)
        print(x_coor, y_coor, 'coordinates')

        black_mask_out = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)
        black_mask_out = cv2.circle(black_mask_out, (int(x_coor), int(y_coor)), 10, (0, 0, 255), 2)

    except Exception as e:
        black_mask_out = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)
        print('e', e)

    end = time.time()

    fps = 1 / (end - start)

    cv2.imwrite('out_1.jpg', black_mask_out)
    cv2.imwrite('raw.jpg', dst)

    print(fps, 'fps')
    print('------')

    rawCapture.truncate(0)