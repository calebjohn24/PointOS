import RPi.GPIO as GPIO
import json
import time

GPIO.setmode(GPIO.BCM)

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


for pin in pin_map['output']:
    GPIO.setup(pin_map['output'][pin], GPIO.OUT)
    GPIO.output(pin_map['output'][pin], GPIO.LOW)


MODE = (motor_0, motor_1, motor_2)
RESOLUTION = {
    '1': (0, 0, 0),
    '1/2': (1, 0, 0),
    '1/4': (0, 1, 0),
    '1/8': (1, 1, 0),
    '1/16': (0, 0, 1),
    '1/32': (1, 0, 1)
}


def set_motor_res(step_res):
    GPIO.output(MODE, RESOLUTION[step_res])


def set_direction(direction):
    if(direction == 'f'):
        GPIO.output(r_dir, 1)
        time.sleep(0.001)
        GPIO.output(l_dir, 0)
        time.sleep(0.001)
        steer_straight()
    elif(direction == 'b'):
        GPIO.output(r_dir, 0)
        time.sleep(0.001)
        GPIO.output(l_dir, 1)
        time.sleep(0.001)
        steer_straight()
    elif(direction == 'r'):
        GPIO.output(r_dir, 0)
        time.sleep(0.001)
        GPIO.output(l_dir, 0)
        time.sleep(0.001)
        steer_turn()
    elif(direction == 'l'):
        GPIO.output(r_dir, 1)
        time.sleep(0.001)
        GPIO.output(l_dir, 1)
        time.sleep(0.001)
        steer_turn()


def move_motors(delay):
    GPIO.output(step, 1)
    time.sleep(delay)
    GPIO.output(step, 0)



def motor_enable():
    GPIO.output(motor_ena, 1)
    time.sleep(0.01)


def motor_disable():
    GPIO.output(motor_ena, 0)
    time.sleep(0.01)


def steer_turn():
    GPIO.output(steer_turn_pin, 1)
    GPIO.output(steer_straight_pin , 0)
    GPIO.output(steer_right_pin, 0)
    GPIO.output(steer_left_pin, 0)
    time.sleep(0.25)


def steer_straight():
    GPIO.output(steer_turn_pin, 0)
    GPIO.output(steer_straight_pin , 1)
    GPIO.output(steer_right_pin, 0)
    GPIO.output(steer_left_pin, 0)
    time.sleep(0.25)

def steer_right():
    GPIO.output(steer_turn_pin, 0)
    GPIO.output(steer_straight_pin , 0)
    GPIO.output(steer_right_pin, 1)
    GPIO.output(steer_left_pin, 0)
    time.sleep(0.25)

def steer_left():
    GPIO.output(steer_turn_pin, 0)
    GPIO.output(steer_straight_pin , 0)
    GPIO.output(steer_right_pin, 0)
    GPIO.output(steer_left_pin, 1)
    time.sleep(0.25)