#!/usr/bin/env python3
import Constants
import color_detection
import arucos_detection
import cv2
import numpy as np
import communication
import camara

def scale_value(c1, c2, pixel, f):
     if (f): # x
          percentage = (pixel + c1.x / 2) / c1.x
          value = percentage * c2.x - c2.x / 2
     else: # y value scale
          percentage = (pixel + c1.y / 2) / c1.y
          value = percentage * c2.y - c2.y / 2
     return value

if __name__ == '__main__':
     # Camara 1
     colors1 = color_detection.ColorDetection()
     arucos1 = arucos_detection.DetectorAruco()
     cam = camara.Camara(Constants.camara_index, colors1, arucos1, False)
     
     # Setup 
     cam.camara_setup()
     
     # Comunication
     arduino = communication.Arduino()
     arduino.connect()

     #Flags
     flag_detect_pattern = False
     flag_lock = False
     find_object = True
     while True:
          # Refresher
          cam.camara_refresh() 
          
          if(flag_detect_pattern):
               xTile = cam.detect_color_pattern()
               if xTile != 7: 
                    print(arduino.sendLocation(xTile))
                    pass
          if(find_object): 
               if(not cam.lock): # base on camara2 
                    # Find closest cube
                    cam.detect_closest_cube()
                    # Lock closest cube
                    cam.lock_object()

               else: 
                    try:
                         lost, following_box = cam.track_object()
                         if(not lost): # only use camara 2
                              print(arduino.cube_found(cam.lock_box[5]))
                              pass
                    except:
                         pass
          
          if cv2.waitKey(1) & 0xFF == ord('q'):
               break
        