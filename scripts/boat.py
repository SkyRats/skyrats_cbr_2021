import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from MRS_MAV import MRS_MAV
import time

class boat_scanner:
    def __init__(self,mav):
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.mav = mav
    
    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
    
    def scan(self, number):
        if number == 0:
            self.mav.set_position(50, -5, 50, hdg = 0)
            time.sleep(15)
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
        if number == 1:
            self.mav.set_position(-45, -5, 50, hdg = 0)
            time.sleep(15)
            rows, cols, a = self.cv_image.shape
            cv2.circle(self.cv_image, (int(cols * 62/100),int(rows * 45/100)), 75, (0,255,0), -1)
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
        #rows, cols, a = self.cv_image.shape
        #self.cv_image = self.cv_image[0 : int(rows), 0 : int(cols)]
        lowerbBaseMovel = np.array([0, 0, 150])
        upperbBaseMovel = np.array([5, 5, 255])
        mask_BaseMovel = cv2.inRange(hsv, lowerbBaseMovel, upperbBaseMovel)

        kernel = np.ones((5,5), np.uint8)
        mask_BaseMovel = cv2.dilate(mask_BaseMovel ,kernel,iterations = 3)
        contours, hierarchy = cv2.findContours(mask_BaseMovel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            M = cv2.moments(cnt)
            try: 
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            except ZeroDivisionError:
                continue
            cv2.drawContours(self.cv_image, contours, -1, (0, 255, 0), 3)
            cv2.imshow('cv_image',self.cv_image)
            cv2.waitKey(15)
            print(cx,cy, np.shape(self.cv_image))

        cv2.imshow('mask_BaseMovel', mask_BaseMovel)
        cv2.waitKey(0)

if __name__ == "__main__":
    rospy.init_node("boat_scanner")
    mav = MRS_MAV("uav1")
    scanner = boat_scanner(mav)
    scanner.scan(0)
    scanner.scan(1)