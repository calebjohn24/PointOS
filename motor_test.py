from PointOS.movement import motor_control
from PointOS.sensors import imu

import RPi.GPIO as GPIO
import json
import threading
import sys
import time


pin_map_file = open('/home/pi/PointOS/res/pinout.json')
pin_map = dict(json.load(pin_map_file))

motor_ena = pin_map['output']['motor_ena']
motor_0 = pin_map['output']['motor_0']
motor_1 = pin_map['output']['motor_1']
motor_2 = pin_map['output']['motor_2']

r_dir = pin_map['output']['r_dir']
l_dir = pin_map['output']['l_dir']

step = pin_map['output']['step']

laser = pin_map['output']['laser']


motor_control.set_direction('f')
motor_control.set_motor_res('1/2')

'''
motor_control.motor_enable()


for i in range(1000):
    motor_control.move_motors(0.0025)

time.sleep(0.2)
motor_control.set_direction('b')
for i in range(1000):
    motor_control.move_motors(0.0025)

time.sleep(0.2)
motor_control.motor_disable()
'''
motor_control.steer_right()
time.sleep(1)
motor_control.steer_straight()
time.sleep(1)
motor_control.steer_left()
time.sleep(1)
motor_control.steer_straight()
time.sleep(1)
GPIO.cleanup()