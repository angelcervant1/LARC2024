import cv2 as cv 
from PIL import Image
import numpy as np
import Constants

def load_value():
    with open(file_name, 'r') as f:
        lines = f.readlines()
        for x in range(0,len(lines)):
            lines[x] = lines[x].replace('[','')
            lines[x] = lines[x].replace(']','')
        lower = np.fromstring(lines[0], dtype=int, sep=",")
        upper = np.fromstring(lines[1], dtype=int, sep=",")
    return lower, upper

camara_index = Constants.camara_index
file_name = Constants.file_name
lowerLimit, upperLimit = load_value()
cap = cv.VideoCapture(camara_index)
while True: 
    ret, frame = cap.read()
    
    hsvImage = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsvImage, lowerLimit, upperLimit)

    mask_ = Image.fromarray(mask)
    bbox = mask_.getbbox()
    
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        frame = cv.rectangle(frame, (x1, y1), (x2, y2), (0,255,255), 5)
    
    cv.imshow("frame", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows