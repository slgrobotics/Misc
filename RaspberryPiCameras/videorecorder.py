import os
import time
import datetime as dt
from picamera import PiCamera
from signal import pause
import tty, sys, termios

destination = '/home/pi/Projects'
filename = ''

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 10
##camera.hflip = True
#camera.awb_mode = 'off'
#rg, bg = (0.5, 0.5)
#rg, bg = (1.3, 1.0)
#rg, bg = (1.2, 0.9)
#camera.awb_gains = (rg, bg)
#camera.exposure_mode = 'off'
#camera.drc_strength='off'


def record_video():
    global filename
    filename = os.path.join(destination, dt.datetime.now().strftime('%Y-%m-%d_%H.%M.%S.h264'))
    print("new file: ", filename)
    camera.start_preview()
    camera.start_recording(filename)

def finish_video():
    camera.stop_recording()
    camera.stop_preview()
    print("closed file: ", filename)

def mygetch():
    import sys, tty, termios
    from select import select
    fd = sys.stdin.fileno()     # does not work in IDLE
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        [i, o, e] = select([sys.stdin.fileno()], [], [], 1)
        if i: ch=sys.stdin.read(1)  # reads 1 char, times out in 1 second
        else: ch=''     
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


try:
    while True:
        record_video()

        for cnt in range (0,180) :
            #time.sleep(1)
            key = mygetch() # times out in 1 second

            # if the `q` key was pressed, exit
            if key == "q":
                print("Key pressed: " + key)
                finish_video()
                exit()

        print("Finished file")
        finish_video()

except KeyboardInterrupt:
    print("End: key pressed")
    finish_video()
    
