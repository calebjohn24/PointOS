import VL53L0X
import sys
import time
import json
import RPi.GPIO as GPIO

pin_map_file = open('/home/pi/PointOS/res/pinout.json')
pin_map = dict(json.load(pin_map_file))


global tof



# Create a VL53L0X object
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

# I2C Address can change before tof.open()
tof.change_address(0x32)
tof.open()
# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)




def get_lidar():
    distance = tof.get_distance()
    return distance
