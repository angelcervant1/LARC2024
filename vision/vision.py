#!/usr/bin/env python3
import Constants
import color_detection
import arucos_detection
import cv2
import numpy as np
import communication
import camara

def scale_value(c1, c2, pixel):
     percentage = (pixel + c1.x / 2) / c1.x
     value = percentage * c2.x - c2.x / 2
     return value

if __name__ == '__main__':
     # Camara 1
     colors1 = color_detection.ColorDetection()
     arucos1 = arucos_detection.DetectorAruco()
     camara1 = camara.Camara(Constants.camara_index, colors1, arucos1, False)

     # Camara 2
     # colors2 = color_detection.ColorDetection()
     # arucos2 = arucos_detection.DetectorAruco()
     # camara2 = camara.Camara(Constants.camara_index2, colors2, arucos2, True)
     
     # Setup 
     camara1.camara_setup()
     # camara2.camara_setup()
     
     # Comunication
     # arduino = communication.Arduino()
     # arduino.connect()

     #Flags
     flag_detect_pattern = False
     flag_lock = False
     find_object = False
     while True:
          # Refresher
          camara1.camara_refresh() 
          # camara2.camara_refresh() 
          
          if(flag_detect_pattern):
               xTile = camara1.detect_color_pattern()
               if xTile != 7: 
                    #Send message with arduino
                    pass
          if(find_object): 
               # Find closest cube
               camara1.detect_closest_cube()

               # Lock closest cube
               camara1.lock_box()
               # camera2.lock_box()
               
               # Find most similar cube
               lost, following_box = camara1.track_object_vertical(camara1.lock_box)
               # camara2.find_specific_cube(following_box[0], scale_value(camara1, camara2, following_box[5]))
               if(not lost):
                    # Check box area
                    pass
               #Send cube data to arduino


          
          if cv2.waitKey(1) & 0xFF == ord('q'):
               break
        
