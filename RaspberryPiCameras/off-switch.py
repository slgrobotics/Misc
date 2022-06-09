from time import sleep
import os
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
btn_timer = 0
while 1:
    state = GPIO.input(21)
    if(state == False):
        btn_timer = btn_timer + 1
    else:
        if(btn_timer > 7):
            os.system('sudo shutdown -h now &')
        btn_timer = 0
    sleep(0.5)
