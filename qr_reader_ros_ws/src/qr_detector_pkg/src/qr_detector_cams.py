#!/usr/bin/env python

from __future__ import print_function
#import roslib
#roslib.load_manifest('algorithm')
import sys
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar


class image_qr:

    def __init__(self):
   
        self.camera_info_high_sub = rospy.Subscriber("/cam_high/camera_info",CameraInfo,self.callback_camera_info_high)
        self.camera_info_low_sub = rospy.Subscriber("/cam_low/camera_info",CameraInfo,self.callback_camera_info_low)
   
        self.image_high_sub = rospy.Subscriber("/cam_high/image_raw",Image,self.callback_high)
        self.image_high_pub = rospy.Publisher("/output_high",Image,queue_size=1)

        self.image_low_sub = rospy.Subscriber("/cam_low/image_raw",Image,self.callback_low)
        self.image_low_pub = rospy.Publisher("/output_low",Image,queue_size=1)

        self.bridge = CvBridge()
        self.K_high = []
        self.D_high = []
        self.K_low = []
        self.D_low = []

    def callback_high(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        draw_img = self.run(cv_image,self.K_high, self.D_high)
        
        try:
            self.image_high_pub.publish(self.bridge.cv2_to_imgmsg(draw_img, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    def callback_low(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        draw_img = self.run(cv_image, self.K_low, self.D_low)
        
        try:
            self.image_low_pub.publish(self.bridge.cv2_to_imgmsg(draw_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def callback_camera_info_high(self,data):
        self.K_high = np.array(data.K,np.float32).reshape(3,3)
        self.D_high = np.array(data.D,np.float32)

    def callback_camera_info_low(self,data):
        self.K_low = np.array(data.K,np.float32).reshape(3,3)
        self.D_low = np.array(data.D,np.float32)
     
    def draw(self,points,im, qr_type, qr_data):
        # If the points do not form a quad, find convex hull
        if len(points) > 4 : 
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else : 
            hull = points;
        
        # Number of points in the convex hull
        n = len(hull)
    
        # Draw the convext hull
        for j in range(0,n):
            cv2.line(im, hull[j], hull[ (j+1) % n], (255,255,0), 3)
            cv2.putText(im, str(j), hull[j], 0, 1, (0, 255, 255), 2)
        
        # between 0 and 2
        center_x = np.absolute(hull[2][0]+hull[0][0])/2
        center_y = np.absolute(hull[2][1]+hull[0][1])/2
        center = (center_x, center_y)
        cv2.putText(im, qr_data, center, 0, 1, (0, 0, 255), 2)

        hull.append(center)
        image_points = np.array(hull,np.float32)
 
        return im, image_points

    def draw_projection(self,img, corners, imgpts):
            corner = tuple(corners[0])
            #print(corner)
            point0 = np.array(imgpts[0])
            point1 = np.array(imgpts[1])
            point2 = np.array(imgpts[2])
            
            img = cv2.line(img, corner, (point0[0][0],point0[0][1]), (255,0,0), 3)
            img = cv2.line(img, corner, (point1[0][0],point1[0][1]), (0,255,0), 3)
            img = cv2.line(img, corner, (point2[0][0],point2[0][1]), (0,0,255), 3)
            return img

    def rot_params_rv(self,rvecs):
        from math import pi,atan2,asin
        R = cv2.Rodrigues(rvecs)[0]
        roll = 180*atan2(-R[2][1], R[2][2])/pi
        pitch = 180*asin(R[2][0])/pi
        yaw = 180*atan2(-R[1][0], R[0][0])/pi
        rot_params= [roll,pitch,yaw]
        return rot_params

    def run(self, img, K, D):

        # Anti orario in metri (40 cm) marker size
        objp = np.array([[0, 0, 0.],  
                    [0, 0.4, 0],       
                    [0.4, 0.4, 0],
                    [0.4, 0, 0],
                    [0.2, 0.2, 0]],np.float32)

        # Find barcodes and QR codes
        decodedObjects = pyzbar.decode(img)
       
        for decodedObject in decodedObjects:
            #print('Type : ', decodedObject.type)
            #print('Data : ', decodedObject.data,'\n')


            points = decodedObject.polygon
            img, image_points = self.draw(points, img, decodedObject.type, decodedObject.data) 

            # Find the rotation and translation vectors.
            
            #print(K)
            #print(D)
            #print(image_points)
            #print(objp)
            
            if len(K)>1:
                ret,rvecs, tvecs = cv2.solvePnP(objp, image_points, K, D)    
           
                rot_params = self.rot_params_rv(rvecs)

                # project 3D points to image plane
                axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, K, D)
             
                
                cv2.putText(img, "x : " + str(round(tvecs[0][0],2)), (0,40), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "y : " + str(round(tvecs[1][0],2)), (0,80), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "z : " + str(round(tvecs[2][0],2)), (0,120), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "roll: " + str(round(rot_params[0],2)), (200,40), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "pitch: " + str(round(rot_params[1],2)), (200,80), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "yaw: " + str(round(rot_params[2],2)), (200,120), 0, 1, (0, 0, 255), 2)           
                img = self.draw_projection(img,image_points,imgpts)
                
        return img;
            
    
def main(args):
    rospy.init_node("qr_detector_node_cams", anonymous=True)

    ic = image_qr()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
