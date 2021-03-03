import RPi.GPIO as GPIO
import json
import threading
import sys
import time
import csv


from PointOS.movement import motor_control
from PointOS.sensors import imu, lidar

filename_out = "training_data/forward_td.csv"


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


global step_count
step_count = [0]
global steer_val
steer_val = [0]
global lidar_data
global lidar_flag

global output_arr
output_arr = []

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
        step_count_val = str(step_count[0])
        steer = str(steer_val[0])
        imu_heading = str(imu_arr[0])
        imu_gyro_x = str(imu_arr[1])
        imu_gyro_y = str(imu_arr[2]) 
        imu_accel_x = str(imu_arr[3]) 
        imu_accel_y = str(imu_arr[4])

        output_arr.append([steer,step_count_val, imu_heading, imu_gyro_x, imu_gyro_y, imu_accel_x, imu_accel_y])

    return



def move_motors():
    while imu_flag == 0:
        motor_control.move_motors(0.00015)
        step_count[0]+= 1
    return





def read_lidars():

    lidar_tgt = int((lidar_data[0] + lidar_data[1])/2)
    while(lidar_flag == 0):
        lidar_data_1 = lidar.get_lidar_1()
        lidar_data_2 = lidar.get_lidar_2()
        lidar_score = int((lidar_data_1 + lidar_data_2)/2)

        current_error = lidar_tgt - lidar_score
        if(current_error > 2):
            motor_control.steer_left()
            steer_val[0] = 1
        elif(current_error < -2):
            motor_control.steer_right()
            steer_val[0] = -1
        else:
            motor_control.steer_straight()
            steer_val[0] = 0

    return


imu_thread = threading.Thread(target=read_imu, args=())
lidar_thread = threading.Thread(target=read_lidars, args=())
imu_thread.start()
lidar_thread.start()



motor_control.set_direction('f')
motor_control.set_motor_res('1/2')
motor_control.motor_enable()


motor_thread = threading.Thread(
    target=move_motors, args=())




print(lidar_data)
print(current_imu_data)
motor_thread.start()

print(lidar_data)
print(current_imu_data)

imu_flag = 1
lidar_flag = 1
motor_thread.join()

motor_control.motor_disable()
imu_thread.join()
lidar_thread.join()



with open(filename_out, 'a') as csvfile:  
    csv_out = csv.writer(csvfile)  
    csv_out.writerows(output_arr) 


GPIO.cleanup()
sys.exit()