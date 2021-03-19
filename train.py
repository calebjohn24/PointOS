import RPi.GPIO as GPIO
import json
import threading
import sys
import time
import csv
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

from PointOS.movement import motor_control
from PointOS.sensors import imu



pin_map_file = open('/home/pi/PointOS/res/pinout.json')
pin_map = dict(json.load(pin_map_file))

motor_ena = pin_map['output']['motor_ena']
motor_0 = pin_map['output']['motor_0']
motor_1 = pin_map['output']['motor_1']
motor_2 = pin_map['output']['motor_2']

r_dir = pin_map['output']['r_dir']
l_dir = pin_map['output']['l_dir']

step = pin_map['output']['step']


steer_straight_pin = pin_map['output']['straight_pin']
steer_right_pin = pin_map['output']['right_pin']
steer_left_pin = pin_map['output']['left_pin']
steer_turn_pin = pin_map['output']['turn_pin']


laser = pin_map['output']['laser']

global imu_flag
imu_flag = [0]


global step_count
step_count = [0]

global output
output = []

camera = PiCamera()
camera.resolution = (600, 600)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(600, 600))


camera.start_preview()
time.sleep(0.1)




def read_imu():
    while imu_flag[0] == 0:
        imu_arr = imu.get_imu_data()
        if(imu_arr[0] != -1 and step_count[0] > 0):

            output.append([step_count[0], imu_arr[0], imu_arr[1], imu_arr[2], imu_arr[3], imu_arr[4]])
    return



def run_camera():
    while imu_flag[0] == 0:
        for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
            img_imm = frame.array
            dst = cv2.cvtColor(img_imm, cv2.COLOR_BGR2RGB)

            dst = dst[400:600, 225:375]

            #dst[:,:,1] = 0
            #dst[:,:,0] = 0

            cv2.imwrite('raw.jpg', dst)

            #print(fps, 'fps')
            print('------')

            rawCapture.truncate(0)
    return


def move_motors_forward():
    motor_control.set_direction('f')
    motor_control.set_motor_res('1/2')
    motor_control.motor_enable()

    for i in range(1000):
        motor_control.move_motors(0.0025)
        step_count[0] = i

    imu_flag[0] = 1
    return





filename = "training_data/forward_td.csv"
     
with open(filename, 'a') as csvfile:  
    csvwriter = csv.writer(csvfile)  
    csvwriter.writerows(output) 
