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
     colors1 = color_detection.ColorDetection(1)
     arucos1 = arucos_detection.DetectorAruco()
     cam = camara.Camara(0, colors1, arucos1, False)

     # Camara 2
     # colors2 = color_detection.ColorDetection(0)
     # arucos2 = arucos_detection.DetectorAruco()
     # cam2 = camara.Camara(1, colors2, arucos2, False)
     
     # Setup 
     cam.camara_setup()
     # cam2.camara_setup()
     
     # Comunication
     # arduino = communication.Arduino()
     # arduino.connect()

     #Flags
     flag_detect_pattern = False
     flag_lock = False
     find_object = False
     ss = time.time()
     iteration = 0
     rotate = False
     angle = 0
     in_front_of_cube = False
     first = True
     tampo = 0 
     while True:
          cam.camara_refresh() 
          # cam.camara_refresh()
          # _, find_object = arduino.get_searching_for_cube()
          # if find_object:
          #      print("Detectado")
          if(rotate):
               # arduino.findOrigin()
               # print(arduino.rotate_90()) 
               time.sleep(3)
               ss = time.time()
               rotate = False
          if(flag_detect_pattern and iteration < 4):
               # arduino.findOrigin()
               if (time.time() - ss > 2):
                    # Camara 2 
                    pass
               if (time.time() - ss > 3):
                    ss = time.time()
                    rotate = True
                    iteration +=1 
               xTile = cam.detect_color_pattern()
               if xTile != 7: 
                    angle = angle * iteration
                    # print(arduino.sendLocation(xTile, angle))
                    print(xTile)
                    flag_detect_pattern = False
                    # find_object = True
                    pass
          elif(find_object):
               # arduino.rotate_90()
               if(not cam.lock): # base on camara2
                    if first:
                         sup = time.time()
                         firts = False 
                    if(time.time() - sup > 5 and cam.lock_box == []):
                         # print(arduino.rotate_90()) 
                         time.sleep(3)
                         sup = time.time()
                         pass
                    # print("Detectado")
                    # Find closest cubeqqq
                    cam.detect_closest_cube()
                    
                    # Lock closest cube
                    cam.lock_object()
                    # print("name" + str(cam.lock_box[0]))
                    #Send data to sencond camara

               else: 
                    # in_front_of_cube = arduino.in_front_of_cube()
                    # print(in_front_of_cubeq)
                    if  (not in_front_of_cube):
                         try:
                              lost, following_box = cam.track_object()
                              # print(arduino.cube_found(int(cam.lock_box[5])))
                              if(not lost): # only use camara 2
                                   if tampo < 4:
                                        in_front_of_cube = True
                                   tampo +=1 
                         except:
                              pass
                    else:
                         # arduino.cube_found(0)
                         # arduino.setCubeDetection() 
                         flag_detect_pattern = True  

                         # print("hook")
                         pass
          
          # Refresher
          if cv2.waitKey(1) & 0xFF == ord('q'):
               break
        