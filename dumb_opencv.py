import numpy as np
import cv2

#cap = cv2.VideoCapture(0)

cap = cv2.VideoCapture('rtsp://admin:cole23tu!@192.168.1.153:554/media/video1') # Test

while(cap.isOpened()):
    ret, frame = cap.read()
    cap.set(3, 1280)
    cap.set(4, 720)
    try:
        cv2.imshow('Frame',frame)
    except:
        print('EOF')
        break

    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()