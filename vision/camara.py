import cv2
import numpy as np
import time

class Camara:
    def __init__(self, camara , colors, arucos, filter):
        self.colors = colors
        self.arucos = arucos
        self.camara_number = camara
        self.cap = cv2.VideoCapture(camara)
        self.box = []
        self.lock_box = []
        self.lock = False
        self.total_boxes = []
        self.ret = False  
        self.frame = np.array([])
        self.image = np.array([])
        self.filter = filter
        self.prev = time.time()
        
    def close(self):
        self.cap.release()
        cv2.destroyWindow(str(self.camara_number))

    
    def open(self):
        self.cap.open(self.camara_number)
        self.camara_setup()
    
    def camara_setup(self):
        self.ret, self.frame = self.cap.read()
        if self.ret:
            if self.filter:
                self.orient_camara()
            self.y, self.x, _= self.frame.shape
            self.arucos.setUp(self.frame)
            self.colors.setUp(self.frame)
        self.reset_values()

    def reset_values(self):
        self.arucos.boxes = []
        self.arucos.detections = []
        self.colors.boxes = []
        self.colors.detections = []
        self.colors.color_close = []
        self.arucos.aruco_detections_data = []
        self.box = []
        self.total_boxes = []

    def camara_refresh(self):
        if (time.time() - self.prev > 0):
            self.prev = time.time()
            # Reset values
            self.reset_values()
            # Capture image
            try: 
                self.ret, self.frame = self.cap.read()
            except:
                pass
            #Verify frame read
            if self.ret:
                if self.filter:
                    self.orient_camara()
                #Join both arucos and colors images 
                self.image = self.arucos.detectar_arucos(self.frame)
                self.image = self.colors.color_detection(self.frame, self.image)
                cv2.imshow(str(self.camara_number), self.image)
                # print(self.ret)
                #Show image
    
    def detect_closest_cube(self):
        #Joins al boxes
        if self.ret:
            if (self.colors.color_close != []):
                total = self.colors.color_close
                closest = 0 
                for index in range(0, len(total)):
                    if total[closest][6] > total[index][6] and total[closest][6] > 400:
                        closest = index
                    elif total[closest][6] == total[index][6]:
                        if total[closest][2] < total[index][2]:
                            closest = index
                # Save it to focus variable
                self.box = total[closest]
    
    def find_specific_cube(self, id, xmid, ymid, mar):
        if self.ret:
            total = self.colors.color_close
            if total != []:
                box = 0
                flag = True
                margin = mar
                for index in range(len(total)):
                    if total[index][0] == id:
                        if flag:
                            box = index
                            flag = False
                        elif abs(total[index][5] - xmid) < abs(total[box][5] - xmid):
                            box = index
                if (int(total[box][5]) in range(int(xmid - margin), int(xmid + margin)) and int(total[box][6]) in range(int(ymid - margin), int(ymid + margin))
                    and total[box][0] == str(id)): # inside x limits and y limit
                    self.box = total[box]
                    #self.image = cv2.drawContours(self.image, (int(xmid - margin), int(ymid + margin)), (int(xmid + margin), int(ymid + margin)), (255,0,0), 3)
                    # print("assdfasdf'")
                    return True, total[box]

                # print("mid " + str(total[box][5]) + " min " + str(int(xmid - margin))+ " max " + str(int(xmid + margin)))
                # print("mid " + str(total[box][6]) + " min " + str(int(ymid - margin))+ " max " + str(int(ymid + margin)))
                return False, []

    def join_arucos_and_color(self):
        if self.ret:
            color = self.colors.color_close
            aruco = self.arucos.aruco_detections_data
            # Check that there is something in either of color or aruco
            if color == None and aruco == None or (len(color) == 0 and len(aruco) == 0):
                return 0
            
            # Joins arrays of data to sort equally
            self.total_boxes = color + aruco
    
    def detect_color_pattern(self):
        if self.ret:
            # Ver como integrarlo con el arduino. Ya que el arduino no espera
            start = time.time()
            while time.time() - start < 4:
                self.camara_refresh()
                self.colors.detect_color_pattern_cb()
                if self.colors.xTile:
                    return self.colors.xTile
                else:
                    return 7 
    
    def orient_camara(self):
        self.frame = cv2.rotate(self.frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    def lock_object(self):
        if self.ret and self.box != []:
            # print("detection" + str(self.box))
            self.lock_box = self.box
            self.lock = True
        else:
            self.lock = False
    
    def track_object(self):
        if self.ret and self.lock_box != []:
            try:
                lost, lock_box_new = self.find_specific_cube(self.lock_box[0], self.lock_box[5], self.lock_box[6], 100)
                if (lost):
                    self.lock_box = lock_box_new
                    #Find closeness with area 
                    area = (self.lock_box[2] - self.lock_box[1]) *  (self.lock_box[4] - self.lock_box[3])
                    # print(area)
                    if area > 95000:
                        return False, self.lock_box 
                    return True, self.lock_box 
                else:
                    # print("out" + str(lost))
                    return False, []
            except:
                pass        