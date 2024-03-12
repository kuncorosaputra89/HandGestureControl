import cv2
import numpy as np
import time
import HandGestureModule as hgm
import math
import osascript
import serial

wCam, hCam = 640, 480
cap = cv2.VideoCapture(1)
cap.set(3, wCam)
cap.set(4, hCam)
prev_time = 0
prevFinger = False
currentFinger = False

detector = hgm.handDetector(detectionCon=0.7)
serial_speed = 115200
serial_port = '/dev/cu.usbmodem1101'
ser = serial.Serial(serial_port, serial_speed, timeout = 1)

while True:
    current_time = time.time()
    ret, frame = cap.read()
    showImage = frame.copy()
    detections = detector.findHands(frame)

    lmList = detector.findPosition(frame, draw=False)
    distance = []
    if (len(lmList) > 0):
        #find thumb distance:
        xBase, yBase, xTip, yTip = lmList[4][1], lmList[4][2], lmList[9][1], lmList[9][2]
        cx,cy = (xBase + xTip)//2, (yBase + yTip)//2
        distance.append(math.sqrt((xTip - xBase)**2 + (yTip - yBase)**2))

        cv2.circle(frame, (xBase,yBase),10,(255,0,0), cv2.FILLED)
        cv2.circle(frame, (xTip, yTip), 10, (255, 0, 0), cv2.FILLED)
        cv2.line(frame, (xBase,yBase), (xTip, yTip), (0, 255, 0), 3)
        cv2.circle(frame, (cx, cy),5,(128,0,128), cv2.FILLED)

        #find distance finger
        for i in range(4):
            xBase, yBase, xTip, yTip = lmList[4*i+5][1], lmList[4*i+5][2], lmList[4*(i+2)][1], lmList[4*(i+2)][2]
            cx, cy = (xBase + xTip) // 2, (yBase + yTip) // 2
            distance.append(math.sqrt((xTip - xBase) ** 2 + (yTip - yBase) ** 2))

            cv2.circle(frame, (xBase, yBase), 10, (255, 0, 0), cv2.FILLED)
            cv2.circle(frame, (xTip, yTip), 10, (255, 0, 0), cv2.FILLED)
            cv2.line(frame, (xBase, yBase), (xTip, yTip), (0, 255, 0), 3)
            cv2.circle(frame, (cx, cy), 5, (128, 0, 128), cv2.FILLED)

        arr = np.array(distance)
        serial_massage = ""

        if(np.max(arr[1:])>120):
            cv2.putText(frame,"Object is too close to your camera", (10,50),cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255),2)
        else:
            vals_greater = (arr[:] > 70.0).sum()
            if np.all(arr[1:] < 70) and arr[0] < 70.0:
                cv2.putText(frame, "LEDS are Turn OFF", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,
                            (255, 255, 255), 2)
                serial_massage = f"Clear"
            elif vals_greater == 1:
                cv2.putText(frame, "ONE LED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,
                            (255, 255, 255), 2)
                serial_massage = f"One"
            elif vals_greater == 2:
                cv2.putText(frame, "TWO LED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,
                            (255, 255, 255), 2)
                serial_massage = f"Two"
            elif vals_greater == 3:
                cv2.putText(frame, "THREE LED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,
                            (255, 255, 255), 2)
                serial_massage = f"Three"
            elif vals_greater == 4:
                cv2.putText(frame, "FOUR LED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,
                            (255, 255, 255), 2)
                serial_massage = f"Four"
            elif vals_greater == 5:
                cv2.putText(frame, "FIVE LED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,
                            (255, 255, 255), 2)
                serial_massage = f"Five"

            print(distance)

        ser.write(serial_massage.encode('utf-8'))
        time.sleep(0.1)
        print(ser.readline().decode('utf-8'))

    fps = 1/(current_time-prev_time)
    previous_time = current_time

    cv2.imshow('frame', frame)
    cv2.waitKey(1)

