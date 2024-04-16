import color_detection
import arucos_detection
import cv2
import numpy as np
import time

class Camara:
    def __init__(self, camara , colors, arucos):
        self.colors = colors
        self.arucos = arucos
        self.cap = cv2.VideoCapture(camara)
        self.box = []
        self.total_boxes = []
        self.ret, self.frame = None
        self.image = np.array([])
    
    def camara_setup(self):
        self.ret, self.frame = self.cap.read()
        if self.ret:
            self.arucos.setUp(self.frame)
            self.colors.setUp(self.frame)
        self.reset_values(self)

    def reset_values(self):
        self.arucos.boxes = []
        self.arucos.detections = []
        self.colors.boxes = []
        self.colors.detections = []
        self.colors.color_close = []
        self.arucos.aruco_detections_data = []

    def camara_refresh(self):
        # Reset values
        self.reset_values()
        
        # Capture image
        self.ret, self.frame = self.cap.read()
        
        #Verify frame read
        if self.ret:
            #Join both arucos and colors images 
            self.image = self.arucos.detectar_arucos(self.frame)
            self.image = self.colors.color_detection(self.frame, self.image)

            #Show image
            # cv2.imshow("frame", self.image)
    
    def detect_closest_cube(self):
        self.join_arucos_and_color()
        total = self.total_boxes
        closest = 0 
        for index in range(len(total)):
            if total[closest][6] < total[index][6]:
                closest = index
            elif total[closest][6] == total[index][6]:
                if total[closest][2] < total[index][2]:
                    closest = index
        # Save it to focus variable
        self.box = total[closest]
    
    def find_specific_cube(self, id, xmid):
        self.join_arucos_and_color()
        total = self.total_boxes
        box = 0
        flag = True
        for index in range(len(total)):
            if total[index][0] == id:
                if flag:
                    box = index
                    flag = False
                elif abs(total[index][5] - xmid) < abs(total[box][5] - xmid):
                    box = index
        if not flag:
            self.box = box

    def join_arucos_and_color(self):
        color = self.colors.color_close
        aruco = self.arucos.aruco_detections_data
        
        # Check that there is something in either of color or aruco
        if color == None and aruco == None or (len(color) == 0 and len(aruco) == 0):
            return 0
        
        # Joins arrays of data to sort equally
        self.total_boxes = color + aruco
    
    def detect_color_pattern(self):
        # Ver como integrarlo con el arduino. Ya que el arduino no espera
        start = time.time()
        while time.time() - start < 4:
            self.camara_refresh()
            self.colors.detect_color_pattern_cb()
            if self.colors.xTile:
                return self.colors.xTile
            else:
                return 7 
