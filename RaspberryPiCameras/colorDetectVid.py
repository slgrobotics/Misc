
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import math
import time
import cv2
import numpy as np
import requests
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)	# yellow LED

# settings:
#serverUrl='http://172.16.1.201:9097'
#serverUrl='http://172.16.1.175:9097'
serverUrl='http://minwinpc:9097'

showframes = True
#showframes = False
 
# initialize the camera and grab a reference to the raw camera capture
#camera = PiCamera()
#camera.resolution = (320, 240)
#camera.framerate = 30
#camera.hflip = True

#rawCapture = PiRGBArray(camera, size=(320, 240))


filename = '/home/pi/Projects/TestVideos/2016-06-12_18.10.02.h264'
#filename = '/home/pi/Projects/TestVideos/2016-06-12_18.09.23.h264'
#filename = '/home/pi/Projects/TestVideos/2016-06-12_18.13.02.h264'

try:
    rawCapture = cv2.VideoCapture(filename)
except:
    print('Could not open video file')


 
# allow the camera to warmup
time.sleep(0.1)

cxPrev = 0
cyPrev = 0

start_time = time.time()
frames = 0
fps = 0

# see http://docs.python-requests.org/en/master/user/advanced/

while True :
  with requests.Session() as s:
    if showframes :
      print("-- new HTTP session --")
    keepsession = True
 
    # capture frames from the camera
    #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    ret = True
    while ret:
        ret, frame = rawCapture.read()
        if ret != True :
           rawCapture.release()
           cv2.destroyAllWindows()
           exit()


        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        #image = frame.array
        image = frame

        blur = cv2.blur(image, (5,5))

        #hsv to complicate things, or stick with BGR
        #hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        #thresh = cv2.inRange(hsv,np.array((0, 200, 200)), np.array((20, 255, 255)))

        #lower = np.array([76,31,4],dtype="uint8")
        ##upper = np.array([225,88,50], dtype="uint8")
        #upper = np.array([210,90,70], dtype="uint8")

        #lower = np.array([0,70,70], dtype="uint8")
        #upper = np.array([20,255,255], dtype="uint8")

        # Lemon Chrome: RGB 255 198 0
        #lower = np.array([0,100,100], dtype="uint8")
        #upper = np.array([100,255,255], dtype="uint8")

        # Red shirt: RGB 255 0 0
        lower = np.array([0,0,120], dtype="uint8")
        upper = np.array([50,80,255], dtype="uint8")

        minarea = 100		# 160 degrees lens
        #minarea = 2500		# normal lens

        thresh = cv2.inRange(blur, lower, upper)
        thresh2 = thresh.copy()

        # find contours in the threshold image
        image,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # finding contour with maximum area and store it as best_cnt
        max_area = 0
        best_cnt = 1
        for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                        max_area = area
                        best_cnt = cnt

        # finding centroids of best_cnt and draw a circle there
        M = cv2.moments(best_cnt)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        if showframes :
                #if best_cnt>1:
                if max_area > minarea and cx != 0 and cy != 0 :
                        cv2.circle(blur,(cx,cy),10,(0,255,255),-1)	# yellow dot
                        #cv2.circle(blur,(cx,cy),10,(0,0,255),-1)	# red dot
                
                # show the frames
                cv2.imshow("Frame", blur)
                cv2.imshow('thresh',thresh2)

        #print(best_cnt, max_area)

        if max_area > minarea and cx != 0 and cy != 0 :
                size = round(math.sqrt(max_area))
                
                GPIO.output(20, False)
                
                if (cx != cxPrev or cy != cyPrev or frames % 5 == 0) :
                        cxPrev = cx
                        cyPrev = cy

                        if showframes :
                                print(cx, cy, size)


        # clear the stream in preparation for the next frame
        #rawCapture.truncate(0)
        frames = frames + 1

        if keepsession == False :
            break

        if showframes :
                key = cv2.waitKey(1) & 0xFF
         
                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                        rawCapture.release()
                        exit()

                # Apparently not supported for my cameras:
                #print ("FPS:", camera.GetCaptureProperty(cap, cv2.CV_CAP_PROP_FPS))
                # print "Frame", frames
                if frames % 100 == 0 :
                        currtime = time.time()
                        numsecs = currtime - start_time
                        fps = frames / numsecs
                        print ("FPS: ", fps)
                        start_time = time.time()
                        frames = 0

