import RPi.GPIO as GPIO
import time

servoPIN = 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(0) # Initialization

time.sleep(0.5)
p.ChangeDutyCycle(12.5)
time.sleep(1)
p.ChangeDutyCycle(9.5)
time.sleep(1)
p.ChangeDutyCycle(12.5)
time.sleep(0.1)
p.stop()
GPIO.cleanup()
