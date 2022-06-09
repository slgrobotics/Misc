from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
while 1:
     GPIO.output(20, False)
     sleep(0.1)
     GPIO.output(20, True)
     sleep(0.1)
