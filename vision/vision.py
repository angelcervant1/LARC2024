#!/usr/bin/env python3
import Constants
import color_detection
import arucos_detection
import cv2
import numpy as np
import communication

flag_detect_pattern = True

def detect_closest_cube(color, aruco):
    if color == None or aruco == None or (len(color) == 0 and len(aruco) == 0):
         return 0
    total = color + aruco
    closest = 0 
    for index in range(len(total)):
        if total[closest][6] < total[index][6]:
             closest = index
        elif total[closest][6] == total[index][6]:
             if total[closest][2] < total[index][2]:
                  closest = index
    return total[closest]

def detect_xtile():
     for i in range(0,4):
        color_detector.detect_color_pattern_cb()
        while arduino.angleOffsetReach() != 0:
            pass
        if(color_detector.xTile != 0 and color_detector.mid_color != 4 and done_rotating):
            flag_detect_pattern = False
            print(color_detector.xTile)
            if(arduino.sendLocation(color_detector.xTile, color_detector.color_tile) == 0):
                color_detector.xTile = 0 
                color_detector.mid_color = 4
                return  1
        else:
            arduino.rotateRobot(90)
            done_rotating = False
        pass
     return 0

if __name__ == '__main__':
    camara_index = Constants.camara_index
    cap = cv2.VideoCapture(camara_index)
    color_detector = color_detection.ColorDetection()
    arucos_detector = arucos_detection.DetectorAruco()
    first_iteration = True
    arduino = communication.Arduino()
    arduino.connect()
    while True: 
        box = []
        ret, frame = cap.read()
        if np.shape(frame) != ():
            if first_iteration:
                arucos_detector.setUp(frame)
                color_detector.setUp(frame)
                first_iteration = False
            
            img = arucos_detector.detectar_arucos(frame)
            img = color_detector.color_detection(frame, img)
            
            # cv2.imshow("frame", img)
            if(flag_detect_pattern):
                 detect_xtile()
            else:
                 box = detect_closest_cube(color_detector.color_close, arucos_detector.aruco_detections_data)
                 if box != 0:
                    arduino.angleOffsetReach(box[5], box[0])
                    pass
            
            arucos_detector.boxes = []
            arucos_detector.detections = []
            color_detector.boxes = []
            color_detector.detections = []
            color_detector.color_close = []
            arucos_detector.aruco_detections_data = []
            if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
