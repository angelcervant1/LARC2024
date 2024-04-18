#!/usr/bin/env python3
import Constants
import color_detection
import arucos_detection
import cv2
import numpy as np
import communication
import camara
import time

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
     cam = camara.Camara(1, colors1, arucos1, False)
     # for i in range(10):
     #      try:
     #           cam = camara.Camara(i, colors1, arucos1, False)
     #      except:
     #           pass
     
     # Setup 
     cam.camara_setup()
     
     # Comunication
     arduino = communication.Arduino()
     arduino.connect()

     #Flags
     flag_detect_pattern = False
     flag_lock = False
     find_object = True
     ss = time.time()
     while True:
          
          if(flag_detect_pattern):
               xTile = cam.detect_color_pattern()
               if xTile != 7: 
                    print(arduino.sendLocation(xTile))
                    flag_detect_pattern = False
                    find_object = True
                    pass
          elif(find_object): 
               if(not cam.lock): # base on camara2 
                    # Find closest cube
                    cam.detect_closest_cube()
                    # Lock cclosest cube
                    cam.lock_object()
                    # print("name" + str(cam.lock_box[0]))

               else: 
                    # print('else')
                    try:
                         lost, following_box = cam.track_object()
                         # print('trying')
                         print(arduino.cube_found(int(cam.lock_box[5])))
                         if(not lost): # only use camara 2
                              pass
                    except:
                         pass
          
          # Refresher
          cam.camara_refresh() 
          if cv2.waitKey(1) & 0xFF == ord('q'):
               break
        