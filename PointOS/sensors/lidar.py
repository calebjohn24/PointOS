import VL53L0X
import sys
import time
import json

pin_map_file = open('/home/pi/PointOS/res/pinout.json')
pin_map = dict(json.load(pin_map_file))


# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = pin_map['output']['lidar0_shutdown']
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = pin_map['output']['lidar1_shutdown']

global tof
global tof1

def setup_sensors():
    GPIO.setup(sensor1_shutdown, GPIO.OUT)
    GPIO.setup(sensor2_shutdown, GPIO.OUT)

    # Set all shutdown pins low to turn off each VL53L0X
    GPIO.output(sensor1_shutdown, 0)
    GPIO.output(sensor2_shutdown, 0)
    time.sleep(0.1)
    GPIO.output(sensor1_shutdown, 0)
    GPIO.output(sensor2_shutdown, 1)
    time.sleep(0.1)


    # Create a VL53L0X object
    tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

    # I2C Address can change before tof.open()
    tof.change_address(0x32)
    tof.open()
    # Start ranging
    tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

    GPIO.output(sensor1_shutdown, 1)
    time.sleep(0.1)
    tof1 = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

    tof1.open()
    # Start ranging
    tof1.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)


def get_both_sensors():
    distance = tof1.get_distance()
    distance1 = tof.get_distance()
    return [distance, distance1]


def get_lidar_1():
    distance = tof.get_distance()
    return distance

def get_lidar_2():
    distance = tof1.get_distance()
    return distance