import RPi.GPIO as GPIO
import json
import threading
import VL53L0X
import sys
import time
import serial


from PointOS.movement import motor_control
from PointOS.sensors import imu, lidar


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
sensor1_shutdown = pin_map['output']['lidar0_shutdown']
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = pin_map['output']['lidar1_shutdown']


'''
imu.open_imu()
print(imu.get_imu_data())
print(imu.get_imu_data())
imu.close_imu()
print('reset IMU')
imu.open_imu()
print(imu.get_imu_data())
print(imu.get_imu_data())
'''


print(lidar.get_both_sensors())
print(lidar.get_lidar_1())
print(lidar.get_lidar_2())

time.sleep(5)

print(lidar.get_both_sensors())

print(lidar.get_lidar_1())
print(lidar.get_lidar_2())



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
