import cv2
import numpy as np
import Constants

class ColorDetection():
    def __init__(self):
        self.boxes = []
        self.cx = 0.0
        self.cy = 0.0
        self.detections = []
        self.image = np.array([])

        self.x_pixels = 0
        self.y_pixels = 0

        self.xTile = 0
        self.color_detections_data = []
        # Flags 
        self.first_iteration = True 
        self.img_flag = True

        self.static_color_seq = "GBYRYBG" # static color sequence
        self.mask  = None
        self.main()
    
    def dibujar(self, mask, color):
        frame = self.image
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=5)
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
       
        temp = []
        for c in contornos:
            self.img_flag = False
            area = cv2.contourArea(c)
            if area > 1000:
                M = cv2.moments(c)
                self.cx = int(M["m10"]/M["m00"]) / self.image.shape[1]
                self.cy = int(M['m01']/M["m00"]) / self.image.shape[0]

                if (M["m00"]):
                    M["m00"] = 1
                x = int(M["m10"]/M["m00"])
                y = int(M['m01']/M["m00"])
                nuevoContorno = cv2.convexHull(c)
                #cv2.circle(frame,(x,y),7,(0,255,0),-1)  
                # cv2.putText(frame,'{},{}'.format(x,y), (x+10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA)
                x, y, w, h = cv2.boundingRect(c)
                xmenor = x
                ymenor = y
                xmayor = x + w
                ymayor = y + h

                temp =  ymenor, xmenor, ymayor, xmayor

                if color == (255,0,0):
                    # print('azul')
                    self.detections.append('azul')
                    
                if color == (0,255,0):
                    # print('verde')
                    self.detections.append('verde')

                if color == (0,0,255):
                    # print('rojo')   
                    self.detections.append('rojo')

                if color == (0,255,255):
                    # print('amarillo')
                    self.detections.append('amarillo')

                self.updated = cv2.drawContours(frame,[nuevoContorno],0,color,3)
                self.boxes.append(temp)

    def get_objects(self, boxes, detections):
        res = []
        #sort boxes full content by first parameter, then second, and save in new obj
        sorted_boxes = []
        sorted_detections = []

        for box in boxes:
            appended = False
            for i in range(len(sorted_boxes)):
                if sorted_boxes[i][1]>box[1] and abs(sorted_boxes[i][0]-box[0])<50 or sorted_boxes[i][0]-box[0]>50:
                    sorted_boxes.insert(i, box)
                    sorted_detections.insert(i, detections[boxes.index(box)])
                    appended = True
                    break
            if not appended:
                sorted_boxes.append(box)
                sorted_detections.append(detections[boxes.index(box)])
        boxes = sorted_boxes
        detections = sorted_detections

        for index in range(len(boxes)):
            # [label, xmin, xmax, pixel en x]
            diference = abs(float(boxes[index][3]) - float(boxes[index][1]))/2
            half = self.x_pixels/2 
            xc = diference + float(boxes[index][1])
            midpoint =  xc - half# (xmax - xmin)/2 + xmin - width
            # print("min: " + str(boxes[index][1]) + " xc: " + str(xc) + "midpoint: " + str(midpoint) + " diference: " + str(diference) + " half " + str(half))
            res.append([str(detections[index]), float(boxes[index][1]), float(boxes[index][3]), midpoint])
        self.color_detections_data = res
        self.detect_color_pattern_cb()

    def color_detection(self):
        frame = self.image
        """
        This can be modify to do it automatic
        """
        lowerRed = np.array([0,162,150], np.uint8)
        upperRed = np.array([7,255,255], np.uint8)
        lowerRed2 = np.array([155,162,150], np.uint8)
        upperRed2 = np.array([179,255,255], np.uint8)
        
        lowerBlue = np.array([  0,187, 32], np.uint8)
        upperBlue = np.array([180,255,255], np.uint8)
        
        lowerYellow = np.array([26,171,168], np.uint8)
        upperYellow = np.array([34,255,255], np.uint8)
        
        lowerGreen = np.array([75,48, 5], np.uint8)
        upperGreen = np.array([104,203, 97], np.uint8)  

        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskAzul = cv2.inRange(frameHSV, lowerBlue, upperBlue)
        maskVerde1 = cv2.inRange(frameHSV, lowerGreen, upperGreen)
        #maskVerde2 = cv2.inRange(frameHSV, lowerGreen2, upperGreen2)
        maskamarillo = cv2.inRange(frameHSV, lowerYellow, upperYellow)
        maskRed1 = cv2.inRange(frameHSV, lowerRed, upperRed)
        maskRed2 = cv2.inRange(frameHSV, lowerRed2, upperRed2)
        maskred = cv2.add(maskRed1,maskRed2)
        #maskverde = cv2.add(maskVerde1,maskVerde2)
        maskverde = maskVerde1
        self.dibujar(maskAzul,(255,0,0))
        self.dibujar(maskamarillo,(0,255,255))
        self.dibujar(maskverde,(0,255,0))
        self.dibujar(maskred,(0,0,255))

        self.get_objects(self.boxes, self.detections)
    
    def detect_color_pattern_cb(self):
        # print("detect_color_pattern_cb")
        data = self.color_detections_data
        xTile = 0

        sz = len(data)
        if sz == 0:
            return xTile

        x_last_max = data[len(data)-1][1]
        point_x_min_id = 0
        color_seq = ""

        color2Letter = {
            "rojo": "R",
            "verde": "G",
            "azul": "B",
            "amarillo": "Y"
        }
    
        for i in range(sz):
            if abs(data[i][1] - x_last_max) >= 130:
                continue
            color_seq += color2Letter[data[i][0]]

            x_last_max = data[i][2]
            #Obtener que color esta en el medio de la imagen
            if( abs(data[i][3]) < abs(data[point_x_min_id][3])):
                point_x_min_id = i

        #check if subsequence
        # print(color_seq)
        #Checa que detecte un minimo de # colores
        if color_seq in self.static_color_seq and len(color_seq) >= 3:
            #get square from closer point x and adjacents
            print(data[point_x_min_id][0])
            x_square_label = color2Letter[ data[point_x_min_id][0] ]
            x_square_cont = ""
            if point_x_min_id > 0:
                x_square_cont = color2Letter[ data[point_x_min_id - 1][0] ] + x_square_label
            else:
                x_square_cont = x_square_label + color2Letter[ data[point_x_min_id + 1][0] ]

            if x_square_label == "G" and x_square_cont == "GB":
                xTile = 7
            elif x_square_label == "B" and (x_square_cont == "GB" or x_square_cont == "BY"):
                xTile = 6
            elif x_square_label == "Y" and (x_square_cont == "BY" or x_square_cont == "YR"):
                xTile = 5
            elif x_square_label == "R":
                xTile = 4
            elif x_square_label == "Y" and (x_square_cont == "RY" or x_square_cont == "YB"):
                xTile = 3
            elif x_square_label == "B" and (x_square_cont == "YB" or x_square_cont == "BG"):
                xTile = 2
            elif x_square_label == "G" and x_square_cont == "BG":
                xTile = 1

            # print("xTile: " + str(xTile))
        self.xTile = xTile
    
    def main(self):
        #Runs camara
        camara_index = Constants.camara_index
        cap = cv2.VideoCapture(camara_index)
        while True: 
            ret, frame = cap.read()
            self.image = frame
            if self.first_iteration:
                y, x, _= frame.shape
                self.x_pixels = x
                self.y_pixels = y
                self.first_iteration = False
            
            if self.img_flag: 
                self.updated = frame

            self.color_detection()
            cv2.imshow("frame", self.updated)
            self.boxes = []
            self.detections = []
            self.img_flag = True
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    


if __name__ == '__main__':
    ColorDetection()