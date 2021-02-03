import time
import serial


global IMU

IMU = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200)

def open_imu():
    IMU = serial.Serial(
        '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200)
    IMU.readline()


def get_imu_data():
    imu_raw = (str(IMU.readline())).split(',')
    imu_raw[0] = imu_raw[0][2:]
    imu_raw[-1] = imu_raw[-1][:-5]
    try:
        heading = float(imu_raw[0])
        gyro_x = float(imu_raw[1])
        gyro_y = float(imu_raw[2])
        accel_x = float(imu_raw[3])
        accel_y = float(imu_raw[4])
        return[heading, gyro_x, gyro_y, accel_x, accel_y]
    except Exception as e:
        print(e)
        return[-1,-1,-1,-1,-1]
    


def close_imu():
    IMU.close()
