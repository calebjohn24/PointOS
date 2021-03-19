
import serial, time
import RPi.GPIO as GPIO




ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200)





while True:
    try:

        for i in range(100):
            st = time.time()
            
            items = str(ser.readline().decode("utf-8") ).split(",")
            del items[-1]
            heading = float(items[0])
            gyro_x = float(items[1])
            gyro_y = float(items[2])
            accel_x = float(items[3])
            accel_y = float(items[4])
            print(heading, gyro_x, gyro_y, accel_x, accel_y)
            
        ser.write(b'r')
        ser.readline()

    except KeyboardInterrupt:
        break