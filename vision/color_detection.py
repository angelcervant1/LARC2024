import cv2
import numpy as np
import Constants

class ColorDetection():
    def __init__(self, id):
        self.boxes = []
        self.cx = 0.0
        self.cy = 0.0
        self.detections = []
        self.image = np.array([])
        self.color_tile = 4
        self.x_pixels = 0
        self.y_pixels = 0

        self.xTile = 7
        self.color_detections_data = []
        self.color_close = []
        # Flags 
        self.first_iteration = True 
        self.img_flag = True

        self.static_color_seq = "GBYRYBG" # static color sequence
        self.mask  = None
        self.mid_color = 4

        # Camara identification
        self.id = id
    
    def dibujar(self, mask, color, img):
        frame = img
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
                self.cx = int(M["m10"]/M["m00"]) / img.shape[1]
                self.cy = int(M['m01']/M["m00"]) / img.shape[0]

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

                img = cv2.drawContours(frame,[nuevoContorno],0,color,3)
                self.boxes.append(temp)
        return img

    def get_objects(self, boxes, detections):
        res = []
        ores = []
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
            """
            y values
            """
            diferencey = abs(float(boxes[index][2])-float(boxes[index][0]))/2
            halfy = self.y_pixels/2
            yc = diferencey + float(boxes[index][0])
            midpointy = yc - halfy
            """ 
            x values
            """
            diference = abs(float(boxes[index][3]) - float(boxes[index][1]))/2
            half = self.x_pixels/2 
            xc = diference + float(boxes[index][1])
            midpoint =  xc - half# (xmax - xmin)/2 + xmin - width
            # print("min: " + str(boxes[index][1]) + " xc: " + str(xc) + "midpoint: " + str(midpoint) + " diference: " + str(diference) + " half " + str(half))
            # [label, xmin, xmax, pixel en x]
            res.append([str(detections[index]), float(boxes[index][1]), float(boxes[index][3]), midpoint])
            # [label, xmin, xmax, ymin, ymax,. xmid, ymid]
            color2Number = {
            "rojo" : 0,
            "verde" : 1,
            "azul" : 2,
            "amarillo" : 3
            }
            ores.append([str(color2Number[detections[index]]), float(boxes[index][1]), float(boxes[index][3]), float(boxes[index][0]), float(boxes[index][2]), midpoint, midpointy ])
        self.color_detections_data = res
        
        self.color_close = ores

    def color_detection(self, img1, img2):
        frame = img1
        """
        This can be modify to do it automatic
        """
        if self.id == 1: # Colores para camara 1
            #Red = Cubo Red2 = Hoja
            

            lowerRed = np.array([147,157, 56], np.uint8)
            upperRed = np.array([180,255,255], np.uint8)
            # lowerRed = np.array([  0,128,119], np.uint8)
            # upperRed = np.array([  8,255,255], np.uint8)
            # lowerRed2 = np.array([0,180,103], np.uint8)
            # upperRed2 = np.array([179,229,152], np.uint8)    


            lowerBlue = np.array([99,77,88], np.uint8)
            upperBlue = np.array([167,255,141], np.uint8)


            lowerYellow = np.array([  0,115,138], np.uint8)
            upperYellow = np.array([180,255,255], np.uint8)





            lowerGreen = np.array([78,62,35], np.uint8)
            upperGreen = np.array([ 92,236,108], np.uint8)
            lowerGreen2 = np.array([37,54,53], np.uint8)
            upperGreen2 = np.array([ 99,177,133], np.uint8)
        else: # Colores para camara 2 
            #Red = Cubo Red2 = Hoja
            

            lowerRed = np.array([  0,183, 122], np.uint8)
            upperRed = np.array([ 3,213,152], np.uint8)
            # lowerRed2 = np.array([0,180,103], np.uint8)
            # upperRed2 = np.array([179,229,152], np.uint8)
            lowerBlue = np.array([22,125,63], np.uint8)
            upperBlue = np.array([108,255,252], np.uint8)
            
            
            lowerYellow = np.array([0,0,0], np.uint8)
            upperYellow = np.array([0,0,0], np.uint8)

            lowerGreen = np.array([0,0,0], np.uint8)
            upperGreen = np.array([0,0,0], np.uint8)

        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskAzul = cv2.inRange(frameHSV, lowerBlue, upperBlue)
        maskVerde1 = cv2.inRange(frameHSV, lowerGreen, upperGreen)
        maskVerde2 = cv2.inRange(frameHSV, lowerGreen2, upperGreen2)
        maskamarillo = cv2.inRange(frameHSV, lowerYellow, upperYellow)
        maskRed1 = cv2.inRange(frameHSV, lowerRed, upperRed)
        # maskRed2 = cv2.inRange(frameHSV, lowerRed2, upperRed2)
        maskred = maskRed1
        maskverde = cv2.add(maskVerde1,maskVerde2)
        maskverde = maskVerde1
        self.dibujar(maskAzul,(255,0,0), img2)
        img2 = self.dibujar(maskamarillo,(0,255,255), img2)
        img2 = self.dibujar(maskverde,(0,255,0), img2)
        img2 = self.dibujar(maskred,(0,0,255), img2)

        self.get_objects(self.boxes, self.detections)
        return img2
    
    def detect_color_pattern_cb(self):
        # print("detect_color_pattern_cb")
        data = self.color_detections_data
        xTile = 0
        # print(data)

        sz = len(data)
        if sz == 0:
            return xTile

        x_last_max = data[0][1]
        point_x_min_id = 0
        color_seq = ""

        color2Letter = {
            "rojo": "R",
            "verde": "G",
            "azul": "B",
            "amarillo": "Y"
        }
        color2Number = {
            "rojo" : 0,
            "verde" : 1,
            "azul" : 2,
            "amarillo" : 3
        }
        for i in range(sz):
            if abs(data[i][1] - x_last_max) >= 130:
                continue
            color_seq += color2Letter[data[i][0]]

            x_last_max = data[i][2]
            #Obtener que color esta en el medio de la imagen
            if( abs(data[i][3]) < abs(data[point_x_min_id][3])):
                point_x_min_id = i
                mid_color = color2Number[data[i][0]]
                self.color_tile = mid_color

        #check if subsequence
        # print(color_seq)
        #Checa que detecte un minimo de # colores
        # print(color_seq)
        if color_seq in self.static_color_seq and len(color_seq) >= 3:
            #get square from closer point x and adjacents
            # print(data[point_x_min_id][0])
            x_square_label = color2Letter[ data[point_x_min_id][0] ]
            x_square_cont = ""
            # print(x_square_label)
            if point_x_min_id > 0:
                x_square_cont = color2Letter[ data[point_x_min_id - 1][0] ] + x_square_label
            else:
                x_square_cont = x_square_label + color2Letter[ data[point_x_min_id + 1][0] ]

            # print(x_square_cont)
            if x_square_label == "G" and x_square_cont == "GB":
                xTile = 0
            elif x_square_label == "B" and (x_square_cont == "GB" or x_square_cont == "BY"):
                xTile = 1
            elif x_square_label == "Y" and (x_square_cont == "BY" or x_square_cont == "YR"):
                xTile = 2
            elif x_square_label == "R":
                xTile = 3
            elif x_square_label == "Y" and (x_square_cont == "RY" or x_square_cont == "YB"):
                xTile = 4
            elif x_square_label == "B" and (x_square_cont == "YB" or x_square_cont == "BG"):
                xTile = 5
            elif x_square_label == "G" and x_square_cont == "BG":
                xTile = 6
            else:
                return 7
        self.xTile = xTile
    
    def setUp(self,img):
        y, x, _= img.shape
        self.x_pixels = x
        self.y_pixels = y

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
    