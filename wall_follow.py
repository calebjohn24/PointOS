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

steer_straight_pin = pin_map['output']['straight_pin']
steer_right_pin = pin_map['output']['right_pin']
steer_left_pin = pin_map['output']['left_pin']
steer_turn_pin = pin_map['output']['turn_pin']


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

global delays
delays = [0.0005,0.0005]# r,l
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



motor_control.set_motor_res('1/2')
motor_control.motor_enable()
start_1 = lidar_data[0]
start_2 = lidar_data[1]


if(start_1 > start_2): #turn right
    motor_control.set_direction('r')
    diff = lidar_data[0] - lidar_data[1]
    if(diff > 1.0):
        while(lidar_data[0] - lidar_data[1] > 0):
            for i in range(25):
                motor_control.move_motors(0.00015)
                
            lidar_data[0] = lidar.get_lidar_1()
            lidar_data[1] = lidar.get_lidar_2()


elif(start_2 > start_1): # turn left
    motor_control.set_direction('l')
    diff = lidar_data[1] - lidar_data[0]
    if(diff > 1.0):
        while(lidar_data[1] - lidar_data[0] > 0):
            for i in range(25):
                motor_control.move_motors(0.00015)

            lidar_data[0] = lidar.get_lidar_1()
            lidar_data[1] = lidar.get_lidar_2()


start_1 = lidar_data[0]
start_2 = lidar_data[1]



def read_imu():
    while(imu_flag == 0):
        imu_arr = imu.get_imu_data()
        current_imu_data[0] = imu_arr[0]
        current_imu_data[1] = imu_arr[1]
        current_imu_data[2] = imu_arr[2]
        current_imu_data[3] = imu_arr[3]
        current_imu_data[4] = imu_arr[4]
    return



def move_motors():
    for i in range(20000):
        motor_control.move_motors(0.00015)
        step_count[0] = i
    return





def read_lidars():
    KP = 0.000000018
    KD = 0.000000004
    KI = 0.00000000002
    lidar_tgt = int((lidar_data[0] + lidar_data[1])/2)
    while(lidar_flag == 0):
        lidar_data_1 = lidar.get_lidar_1()
        lidar_data_2 = lidar.get_lidar_2()
        lidar_score = int((lidar_data_1 + lidar_data_2)/2)

        current_error = lidar_tgt - lidar_score
        adj = (current_error * KP) + (prev_errors[0] * KD) + (prev_errors[1] * KI)
        if(adj > 0): # slow down right motor
            if(delays[1] > 0.0002):# max left motor speed
                delays[1] -= adj
                delays[0] += adj
        if(adj < 0):# slow down left motor
            if(delays[0] > 0.0002):# max right motor speed
                delays[0] -= adj
                delays[1] += adj

        print(current_error)
        print(str(delays[1]) + ' l')
        print(str(delays[0]) + ' r')

    return


imu_thread = threading.Thread(target=read_imu, args=())
lidar_thread = threading.Thread(target=read_lidars, args=())
imu_thread.start()
lidar_thread.start()



motor_control.set_direction('f')
motor_control.set_motor_res('1/4')
motor_control.motor_enable()


motor_thread = threading.Thread(
    target=move_motors, args=())




print(lidar_data)
print(current_imu_data)
motor_thread.start()
motor_thread.join()

motor_control.motor_disable()
print(lidar_data)
print(current_imu_data)

imu_flag = 1
lidar_flag = 1
imu_thread.join()
lidar_thread.join()


GPIO.cleanup()
sys.exit()