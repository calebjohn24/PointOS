import RPi.GPIO as GPIO
import json

import threading



from PointOS.movement import motor_control


pin_map_file = open('/home/pi/PointOS/res/pinout.json')
pin_map = dict(json.load(pin_map_file))

motor_ena = pin_map['output']['motor_ena']
motor_0 = pin_map['output']['motor_0']
motor_1 = pin_map['output']['motor_1']
motor_2 = pin_map['output']['motor_2']

r_dir = pin_map['output']['r_dir']
l_dir = pin_map['output']['l_dir']

r_step = pin_map['output']['r_step']
l_step = pin_map['output']['l_step']

laser = pin_map['output']['laser']



motor_control.set_direction('r')
motor_control.set_motor_res('1/2')

for i in range(1000):
    motor_control.move_left_motor(0.001)
    motor_control.move_right_motor(0.001)


GPIO.cleanup()