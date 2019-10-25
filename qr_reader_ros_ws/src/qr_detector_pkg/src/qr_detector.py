#!/usr/bin/env python

from __future__ import print_function
#import roslib
#roslib.load_manifest('algorithm')
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar


class image_qr:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("image_result",Image,queue_size=1)
        self.bridge = CvBridge()

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        draw_img = self.run(cv_image)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(draw_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

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
            cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)
        
        cv2.putText(im, qr_type + ": " + qr_data, hull[0], 0, 0.8, (0, 255, 0), 2)
 
        return im

    def run(self, img):
        
        # Find barcodes and QR codes
        decodedObjects = pyzbar.decode(img)
       
        for decodedObject in decodedObjects:
            print('Type : ', decodedObject.type)
            print('Data : ', decodedObject.data,'\n')

            points = decodedObject.polygon
            img = self.draw(points, img, decodedObject.type, decodedObject.data) 

        return img;
            
    
def main(args):
    rospy.init_node('qr_detector_node', anonymous=True)
    ic = image_qr()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
