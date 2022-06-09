from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
GPIO.setup(20, GPIO.OUT)
while 1:
     state = GPIO.input(21)
     GPIO.output(20, state)
     sleep(0.1)
