import cv2
import numpy as np
import pathlib
import sys

sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *

class DetectorAruco:
    def __init__(self):
        self.cx = 0
        self.cy = 0

        self.flag = False
        self.mask  = None
        self.cv_image = np.array([])
        self.main()


    def detectar_arucos(self):
        frame = self.cv_image
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        frame_with_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.cv_image = frame_with_markers

        bb = []
        detections = []
        tempo = []
        if ids is not None:
            if corners:
                for i, marker_corners in enumerate(corners):
                    print(ids[i])
                    corner = corners[i][0]
                    xmayor = np.amax(corner[:, 0])
                    ymayor = np.amax(corner[:, 1])
                    xmenor = np.amin(corner[:, 0])
                    ymenor = np.amin(corner[:, 1])
                    #get x and y centroid
                    self.cx = (xmayor + xmenor)/2 / self.cv_image.shape[1]
                    self.cy = (ymayor + ymenor)/2 / self.cv_image.shape[0]

                    # draw a point on the centroid
                    cv2.circle(self.cv_image, (int(self.cx * self.cv_image.shape[1]), int(self.cy * self.cv_image.shape[0])), 5, (0, 0, 255), -1)

                    #print(f"Xmayor: {xmayor:.2f}, Xmenor: {xmenor:.2f}, Ymayor: {ymayor:.2f}, Ymenor: {ymenor:.2f}")    
                    tempo =  ymenor, xmenor, ymayor, xmayor
                    bb.append(tempo)

                    #Remove brackets from ids
                    tmp_d = str(ids[i]).strip('[]')
                    detections.append(tmp_d)
                    #ids[i][j] Es el id del aruco
                    #corners Es la bounding box del aruco
            self.get_objects(bb, detections)

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
    # def get_objects(self, boxes, detections):
    #     res = []

    #     pa = PoseArray()
    #     pa.header.frame_id = "zed2_base_link"
    #     pa.header.stamp = rospy.Time.now()
    #     for index in range(len(boxes)):
    #         if True:
    #             point3D = Point()
    #             rospy.logwarn("---------------------------")


    #             rospy.logwarn("pose")
    #             point2D  = get2DCentroid(boxes[index])
    #             rospy.logwarn(point2D)
    #             # Dummy point2d

    #             if len(self.depth_image) != 0:
    #                 depth = get_depth(self.depth_image, point2D)
    #                 point3D_ = deproject_pixel_to_point(self.camera_info, point2D, depth)
    #                 point3D.x = point3D_[0] - 0.05
    #                 point3D.y = point3D_[1]
    #                 point3D.z = point3D_[2]
    #                 pa.poses.append(Pose(position=point3D))
    #                 res.append(
    #                 objectDetection(

    #                     label = int(index), # 1
    #                     labelText = str(detections[index]), # "Hscore = float(0.0),
    #                     category = str('aruco'),
    #                     cx = float(self.cx),
    #                     cy = float(self.cy),    
    #                     ymin = float(boxes[index][0]),
    #                     xmin = float(boxes[index][1]),
    #                     ymax = float(boxes[index][2]),
    #                     xmax = float(boxes[index][3]),
    #                     point3D = point3D
    #                 )
    #             )
    #         self.posePublisher.publish(pa)

    #     self.pubData.publish(objectDetectionArray(detections=res))                

        
    
    def main(self):
        rospy.logwarn("Starting listener")
        rospy.init_node('detector_arucos', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        try :
            while not rospy.is_shutdown():
                
                rate.sleep()
        except KeyboardInterrupt:
            rospy.logwarn("Keyboard interrupt detected, stopping listener")


if __name__ == '__main__':
    DetectorAruco()