import RPi.GPIO as GPIO
import json
import threading
import sys
import time


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


global current_imu_data
current_imu_data = [-1.0, -1.0, -1.0, -1.0, -1.0]
#Heading, gyro_x, gyro_y, accel_x, accel_y

global imu_flag
imu_flag = 0

global delay_r
global delay_l
delay_r = 0.0005
delay_l = 0.0005

global step_count
step_count = [0,0]#0 index Left, 1 index Right

global lidar_data
global lidar_flag

lidar_data = [-1.0, -1.0]
lidar_flag = 0

lidar_data[0] = lidar.get_lidar_1()# front sensor
lidar_data[1] = lidar.get_lidar_2()# back sensor

lidar_data[0] = lidar.get_lidar_1()
lidar_data[1] = lidar.get_lidar_2()

global prev_errors

prev_errors = [0.0,0.0] #0 Index is last error, 1 Index is cum error

imu.open_imu()

current_imu_data = imu.get_imu_data()



motor_control.set_motor_res('1/4')
motor_control.motor_enable()
start_1 = lidar_data[0]
start_2 = lidar_data[1]


if(start_1 > start_2): #turn left
    motor_control.set_direction('l')
    diff = lidar_data[0] - lidar_data[1]
    if(diff > 1.0):
        while(lidar_data[0] - lidar_data[1] > 1.0):
            for i in range(1000):
                motor_control.move_right_motor(0.0006)
                motor_control.move_left_motor(0.0006)
            lidar_data[0] = lidar.get_lidar_1()
            lidar_data[1] = lidar.get_lidar_2()


elif(start_2 > start_1): # turn right
    motor_control.set_direction('r')
    diff = lidar_data[1] - lidar_data[0]
    if(diff > 1.0):
        while(lidar_data[1] - lidar_data[0] > 1.0):
            for i in range(1000):
                motor_control.move_right_motor(0.0006)
                motor_control.move_left_motor(0.0006)
            lidar_data[0] = lidar.get_lidar_1()
            lidar_data[1] = lidar.get_lidar_2()


start_1 = lidar_data[0]
start_2 = lidar_data[1]



def move_right_motor():
    for i in range(2000):
        motor_control.move_right_motor(delay_r)
        step_count[0] = i
    return


def move_left_motor():
    for i in range(2000):
        motor_control.move_left_motor(delay_l)
        step_count[1] = i
    return


def read_imu():
    while(imu_flag == 0):
        imu_arr = imu.get_imu_data()
        current_imu_data[0] = imu_arr[0]
        current_imu_data[1] = imu_arr[1]
        current_imu_data[2] = imu_arr[2]
        current_imu_data[3] = imu_arr[3]
        current_imu_data[4] = imu_arr[4]
    return



KP = 0.00001
KD = 0.0000025
KI = 0.0000001


def read_lidars():
    while(lidar_flag == 0):
        lidar_data[0] = lidar.get_lidar_1()
        lidar_data[1] = lidar.get_lidar_2()
        current_error = lidar_data[0] - lidar_data[1]
        adj = (current_error * KP) + (prev_errors[0] * KD) + (prev_errors[1] * KI)
        if(adj > 0.0): # adj left +
            delay_l = delay_l + adj
            delay_r = delay_r - adj
            prev_errors[1] += current_error
            prev_errors[0] = current_error
        elif(adj < 0.0): # adj right +
            delay_r = delay_r + adj
            delay_l = delay_l - adj
            prev_errors[1] += current_error
            prev_errors[0] = current_error

        time.sleep(0.01)
    return


imu_thread = threading.Thread(target=read_imu, args=())
lidar_thread = threading.Thread(target=read_lidars, args=())
imu_thread.start()
lidar_thread.start()



motor_control.set_direction('f')
motor_control.set_motor_res('1/4')
motor_control.motor_enable()


right_motor_thread = threading.Thread(
    target=move_right_motor, args=())
left_motor_thread = threading.Thread(
    target=move_left_motor, args=())



print(lidar_data)
print(current_imu_data)
right_motor_thread.start()
left_motor_thread.start()
right_motor_thread.join()
left_motor_thread.join()
motor_control.motor_disable()
print(lidar_data)
print(current_imu_data)

imu_flag = 1
lidar_flag = 1
imu_thread.join()
lidar_thread.join()




GPIO.cleanup()
