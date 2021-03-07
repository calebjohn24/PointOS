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

    dst = dst[400:600, 225:375]

    #dst[:,:,1] = 0
    #dst[:,:,0] = 0

    cv2.imwrite('raw.jpg', dst)

    #print(fps, 'fps')
    print('------')

    rawCapture.truncate(0)