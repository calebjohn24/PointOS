import RPi.GPIO as GPIO
import json
import threading
import VL53L0X
import time, sys


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


# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 23
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 18


GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)
time.sleep(0.1)
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.HIGH)
time.sleep(0.1)


# Create a VL53L0X object
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

# I2C Address can change before tof.open()
tof.change_address(0x32)
tof.open()
# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

GPIO.output(sensor1_shutdown, GPIO.HIGH)
time.sleep(0.1)
tof1 = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

tof1.open()
# Start ranging
tof1.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)



for count in range(1, 101):
    distance = tof1.get_distance()
    distance1 = tof.get_distance()
    print("s0: " + str(distance) + " mm")
    print("s1: " + str(distance1) + " mm")

    time.sleep(0.001)

tof.stop_ranging()
tof.close()
    
tof1.stop_ranging()
tof1.close()




motor_control.set_direction('b')
motor_control.set_motor_res('1/2')



'''
motor_control.motor_enable()

for i in range(10000):
    motor_control.move_left_motor(0.00015)

for i in range(10000):
    motor_control.move_right_motor(0.00015)

motor_control.motor_disable()
'''



GPIO.cleanup()