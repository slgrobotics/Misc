import numpy as np
import cv2

# must load V4L2 driver:
# sudo modprobe bcm2835-v412

cap = cv2.VideoCapture(-1)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # print (frame.shape)

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
