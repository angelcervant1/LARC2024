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
     find_object = True
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
               if(not camara1.lock): # base on camara2 
                    # Find closest cube
                    camara1.detect_closest_cube()
                    # Lock closest cube
                    camara1.lock_object()
                    # xmid = scale_value(camara1, camara2, camara1.lock_box[5], True)
                    # ymid = scale_value(camara1, camara2, camara1.lock_box[6], False)
                    # try:
                    #      flag2, camara2.box  = camera2.find_specific_cube(camera1.lock_box[0], xmid, ymid, 100) # Find same cube on with the bottom camara
                    #      if (flag2):
                    #         camara2.lock_object()
                    # except:
                    #      pass
               else: 
                    # Find most similar cube
                    try:
                         lost, following_box = camara1.track_object() # this needs to be camara 2
                         if(not lost): # only use camara 2
                              # grab
                              # print("camara 2 ")
                              pass
                    except:
                         pass
          
          if cv2.waitKey(1) & 0xFF == ord('q'):
               break
        
