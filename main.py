import RPi.GPIO as GPIO
import json

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
    GPIO.output(pin_map['output'][pin], GPIO.LOW)


