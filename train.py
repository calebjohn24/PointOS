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

global steer_dir
steer_dir = [1]

global output
output = []


camera = PiCamera()
camera.resolution = (600, 600)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(600, 600))


camera.start_preview()
time.sleep(0.1)
imu.open_imu()

def read_imu():
    while imu_flag[0] == 0:
        imu_arr = imu.read_imu()
        if(imu_arr[0] != -1 and step_count[0] > 0):

            output.append([steer_dir[0], step_count[0], imu_arr[0],
                           imu_arr[1], imu_arr[2], imu_arr[3], imu_arr[4]])
    return


def run_camera_forward():
    frame_count = 0
    x_tgt = 0
    while imu_flag[0] == 0:
        for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
            img_imm = frame.array
            dst = cv2.cvtColor(img_imm, cv2.COLOR_BGR2RGB)

            dst = dst[350:600, 150:450]
            dst[:, :, 2] = 0
            dst[:, :, 1] = 0

            hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

            lower_range = np.array([60,60,60])
            upper_range = np.array([255,255,255])

            mask = cv2.inRange(hsv, lower_range, upper_range)

            indices = np.where(mask != [0])
            try:

                x_coordinates = indices[1]

                x_coordinates = np.sort(x_coordinates)

                x_coor = np.median(x_coordinates)

                y_coordinates = indices[0]

                y_coordinates = np.sort(y_coordinates)

                y_coor = np.median(y_coordinates)
                #print(x_coor, y_coor, 'coordinates')

                if(frame_count == 0):
                    x_tgt = x_coor

                frame_count += 1

                if(x_coor - x_tgt > 5):
                    motor_control.steer_right()
                    steer_dir[0] = 2
                elif(x_coor - x_tgt < -5):
                    motor_control.steer_left()
                    steer_dir[0] = 0
                else:
                    motor_control.steer_straight()
                    steer_dir[0] = 1

                mask_out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                mask_out = cv2.circle(
                    mask_out, (int(x_coor), int(y_coor)), 10, (0, 0, 255), 2)

            except Exception as e:
                mask_out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                print('e', e)
                imu_flag[0] = 1
                rawCapture.truncate(0)
                return

            cv2.imwrite('raw.jpg', dst)
            cv2.imwrite('mask.jpg', mask_out)

            

            #print(fps, 'fps')
            # print('------')

            rawCapture.truncate(0)

            if(imu_flag[0] == 1):
                return
    return


def run_camera_backward():
    frame_count = 0
    x_tgt = 0
    while imu_flag[0] == 0:
        for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
            img_imm = frame.array
            dst = cv2.cvtColor(img_imm, cv2.COLOR_BGR2RGB)

            dst = dst[350:600, 150:450]
            dst[:, :, 2] = 0
            dst[:, :, 1] = 0

            hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

            lower_range = np.array([40,40,40])
            upper_range = np.array([255,255,255])

            mask = cv2.inRange(hsv, lower_range, upper_range)

            indices = np.where(mask != [0])
            try:

                x_coordinates = indices[1]

                x_coordinates = np.sort(x_coordinates)

                x_coor = np.median(x_coordinates)

                y_coordinates = indices[0]

                y_coordinates = np.sort(y_coordinates)

                y_coor = np.median(y_coordinates)
                #print(x_coor, y_coor, 'coordinates')

                if(frame_count == 0):
                    x_tgt = x_coor

                frame_count += 1

                if(x_coor - x_tgt > 5):
                    motor_control.steer_left()
                    steer_dir[0] = 2
                elif(x_coor - x_tgt < -5):
                    motor_control.steer_right()
                    steer_dir[0] = 0
                else:
                    motor_control.steer_straight()
                    steer_dir[0] = 1

                mask_out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                mask_out = cv2.circle(
                    mask_out, (int(x_coor), int(y_coor)), 10, (0, 0, 255), 2)

            except Exception as e:
                mask_out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                print('e', e)
                imu_flag[0] = 1
                rawCapture.truncate(0)
                return

            cv2.imwrite('raw.jpg', dst)
            cv2.imwrite('mask.jpg', mask_out)

            

            #print(fps, 'fps')
            # print('------')

            rawCapture.truncate(0)
            if(imu_flag[0] == 1):
                return
    return


def move_motors_forward():
    motor_control.set_direction('f')
    motor_control.set_motor_res('1/2')
    motor_control.motor_enable()

    for i in range(12000):
        if(imu_flag[0] == 0):
            motor_control.move_motors(0.0025)
            step_count[0] = i
        else:
            return
    time.sleep(0.1)
    motor_control.motor_disable()
    imu_flag[0] = 1
    return

def move_motors_backward():
    motor_control.set_direction('b')
    motor_control.set_motor_res('1/2')
    motor_control.motor_enable()

    for i in range(12000):
        if(imu_flag[0] == 0):
            motor_control.move_motors(0.0025)
            step_count[0] = i
        else:
            return
    time.sleep(0.1)
    motor_control.motor_disable()
    imu_flag[0] = 1
    return


imu_thread_forward = threading.Thread(target=read_imu, args=())
motor_thread_forward = threading.Thread(
    target=move_motors_forward, args=())
cam_thread_forward = threading.Thread(
    target=run_camera_forward, args=())


imu_thread_forward.start()

cam_thread_forward.start()
time.sleep(0.2)
motor_thread_forward.start()

imu_thread_forward.join()
motor_thread_forward.join()
cam_thread_forward.join()

imu.reset_imu()


filename = "training_data/forward_td.csv"

with open(filename, 'a') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerows(output)


output = []
imu_flag[0] = 0

imu_thread_backward = threading.Thread(target=read_imu, args=())
motor_thread_backward = threading.Thread(
    target=move_motors_backward, args=())
cam_thread_backward = threading.Thread(
    target=run_camera_backward, args=())


imu_thread_backward.start()

cam_thread_backward.start()

time.sleep(0.2)
motor_thread_backward.start()

imu_thread_backward.join()
motor_thread_backward.join()
cam_thread_backward.join()



filename = "training_data/backward_td.csv"

with open(filename, 'a') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerows(output)