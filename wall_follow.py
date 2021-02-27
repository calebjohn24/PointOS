import RPi.GPIO as GPIO
import json
import threading
import sys
import time


from PointOS.movement import motor_control
from PointOS.sensors import imu


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

global delays
delays = [0.00015,0.00015]# r,l
delay_r = 0.0005
delay_l = 0.0005

global step_count
step_count = [0,0]#0 index Left, 1 index Right



global prev_errors

prev_errors = [0.0,0.0] #0 Index is last error, 1 Index is cum error

imu.open_imu()

current_imu_data = imu.get_imu_data()



motor_control.set_motor_res('1/2')
motor_control.motor_enable()
def read_imu():
    while(imu_flag == 0):
        imu_arr = imu.get_imu_data()
        current_imu_data[0] = imu_arr[0]
        current_imu_data[1] = imu_arr[1]
        current_imu_data[2] = imu_arr[2]
        current_imu_data[3] = imu_arr[3]
        current_imu_data[4] = imu_arr[4]
    return



def move_right_motor():
    for i in range(60000):
        motor_control.move_right_motor(delays[0])
        step_count[0] = i
    return


def move_left_motor():
    for i in range(60000):
        motor_control.move_left_motor(delays[1])
        step_count[1] = i
    return





imu_thread = threading.Thread(target=read_imu, args=())

imu_thread.start()




motor_control.set_direction('b')
motor_control.set_motor_res('1/2')
motor_control.motor_enable()

right_motor_thread = threading.Thread(
    target=move_right_motor, args=())
left_motor_thread = threading.Thread(
    target=move_left_motor, args=())


print(current_imu_data)
right_motor_thread.start()
left_motor_thread.start()

right_motor_thread.join()
left_motor_thread.join()
motor_control.motor_disable()
imu_flag = 1

print(current_imu_data)

print(step_count)
imu_thread.join()



GPIO.cleanup()
sys.exit()
