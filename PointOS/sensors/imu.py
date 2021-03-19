import time
import serial


global IMU

IMU = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200)

def open_imu():
    if(not IMU.is_open):
        IMU.open()
        IMU.readline()
    IMU.readline()


def read_imu():
    imu_raw = str(IMU.readline().decode("utf-8") ).split(",")
    del imu_raw[-1]
    try:
        heading = float(imu_raw[0])
        gyro_x = float(imu_raw[1])
        gyro_y = float(imu_raw[2])
        accel_x = float(imu_raw[3])
        accel_y = float(imu_raw[4])
        return[heading, gyro_x, gyro_y, accel_x, accel_y]
    except Exception as e:
        print(e)
        print("bad IMU data ignore")
        IMU.readline()
        return[-1,-1,-1,-1,-1]
    


def close_imu():
    if(IMU.is_open):
        IMU.close()


def reset_imu():
    IMU.write(b'r')
    IMU.readline()