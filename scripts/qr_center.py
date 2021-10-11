#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from rospy.core import is_shutdown
from rospy.topics import Publisher
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge,CvBridgeError
import time
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

########## DETECTA QRCODE  ################################################

class qrcode_finder:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.cv_state = rospy.Subscriber("/qrcode_finder/set_running_state", Bool, self.cv_callback)
        self.qrcode_center_publisher = rospy.Publisher("/qrcode_finder/center", Point, queue_size=10)

        self.bridge_object = CvBridge()
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.rate = rospy.Rate(60)
        self.qrcode_center = Point()


    def cv_callback(self, data):
        self.cv = data.data

    def camera_callback(self, data):
        cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.rows, self.cols, a = cv_image.shape
        self.hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)


    def detector(self):

        lowerbbranco = np.array([0, 0, 250])
        upperbbranco = np.array([5, 5, 255])
        mask_branco = cv2.inRange(self.hsv, lowerbbranco, upperbbranco)    

        contours, hierarchy = cv2.findContours(mask_branco, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            rospy.logwarn("Mais de um contorno")
        elif len(contours) == 0:
            self.qrcode_center.z = 0
        else:
            for cnt in contours:
                M = cv2.moments(cnt)
                try: 
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.qrcode_center.x = cx
                    self.qrcode_center.y = cy
                    self.qrcode_center.z = 0
                   
                    print(cx)
                    print(cy)
                except ZeroDivisionError:
                    pass

            for b in range(40):
                self.qrcode_center_publisher(self.qrcode_center)
                self.rate.sleep()
            cv2.drawContours(self.hsv, contours, -1, (0, 255, 0), 3)
            cv2.imshow('hsv',self.hsv)
            cv2.waitKey(0)


    def run(self):
        while(not rospy.is_shutdown()):
            if self.cv:
                self.detector()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('qrcode_finder_node')
    a = qrcode_finder()
    a.run()