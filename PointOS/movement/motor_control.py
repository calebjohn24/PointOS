import RPi.GPIO as GPIO
import json
import time

GPIO.setmode(GPIO.BCM)

pin_map_file = open('res/pinout.json')
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


for pin in pin_map['output']:
    GPIO.setup(pin_map['output'][pin], GPIO.OUT)
    GPIO.output(pin_map['output'][pin], 0)

MODE = (motor_0, motor_1, motor_2)   # Microstep Resolution GPIO Pins
RESOLUTION = {'Full': (0, 0, 0),
              'Half': (1, 0, 0),
              '1/4': (0, 1, 0),
              '1/8': (1, 1, 0),
              '1/16': (0, 0, 1),
              '1/32': (1, 0, 1)}


r_speed = 50
l_speed = 50

delay_const = 0.00001

def set_motor_res(step_res):
    GPIO.output(MODE, RESOLUTION[step_res])


def set_direction(direction):
    if(direction == 'f'):
        GPIO.output(r_dir, 1)
        time.sleep(0.001)
        GPIO.output(l_dir, 0)
        time.sleep(0.001)
    elif(direction == 'b'):
        GPIO.output(r_dir, 0)
        time.sleep(0.001)
        GPIO.output(l_dir, 1)
        time.sleep(0.001)
    elif(direction == 'r'):
        GPIO.output(r_dir, 0)
        time.sleep(0.001)
        GPIO.output(l_dir, 0)
        time.sleep(0.001)
    elif(direction == 'l'):
        GPIO.output(r_dir, 1)
        time.sleep(0.001)
        GPIO.output(l_dir, 1)
        time.sleep(0.001)

def move_right_motor(delay):
    GPIO.output(r_step, 1)
    time.sleep(delay)
    GPIO.output(r_step, 0)


def move_left_motor(delay):
    GPIO.output(l_step, 1)
    time.sleep(delay)
    GPIO.output(l_step, 0)
