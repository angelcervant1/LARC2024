#!/usr/bin/env python3
import Constants
import color_detection
import arucos_detection
import cv2
import numpy as np
import communication
import camara

def scale_value(c1, c2):
     if np.shape(c1.box) == ():
           return []
     y, x, _= c1.frame.shape
     y, x2, _= c2.frame.shape
     percentage = (c1.box[5] + x/2)/ x
     value = percentage * x2 - x2/2
     return value

if __name__ == '__main__':
     # Camara 1
     colors1 = color_detection.ColorDetection()
     arucos1 = arucos_detection.DetectorAruco()
     camara1 = camara.Camara(Constants.camara_index, colors1, arucos1)

     # Camara 2
     colors2 = color_detection.ColorDetection()
     arucos2 = arucos_detection.DetectorAruco()
     camara2 = camara.Camara(Constants.camara_index2, colors2, arucos2)
     
     # Setup 
     camara1.camara_setup()
     camara2.camara_setup()
     
     # Comunication
     arduino = communication.Arduino()
     arduino.connect()

     #Flags
     flag_detect_pattern = True
     while True:
          # Refresher
          camara1.camara_refresh() 
          camara2.camara_refresh() 
          
          if(flag_detect_pattern):
               xTile = camara1.detect_color_pattern()
               if xTile != 7: 
                    #Send message with arduino
                    pass
               else: 
                    camara1.detect_closest_cube()
                    camara2.find_specific_cube(camara1.box[0], scale_value(camara1, camara2))
          
               if cv2.waitKey(1) & 0xFF == ord('q'):
                         break
        
