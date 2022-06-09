import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.output(20, True)
