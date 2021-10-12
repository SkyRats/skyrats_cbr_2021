#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from MRS_MAV import MRS_MAV
from std_msgs.msg import Bool

import time
MIN_LAR = 1200

class fase1:
    def __init__(self,mav):
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.land_sub = rospy.Subscriber("/precision_land/land", Bool, self.land_callback)

        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.mav = mav
        self.bases_moveis =[]
        self.encontrou_lar = 0
        self.land = 0
        self.soma = 0
        self.rate = rospy.Rate(60)


        # Cam Params
        self.image_pixel_width = 752
        self.image_pixel_height = 480

    def land_callback(self, data):
        self.land = data.data

    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")

    def tubo(self):
        self.rows, self.cols, b = self.cv_image.shape
        self.cv_image_cortada = self.cv_image[0: int(self.rows - (self.rows*0.2)) , int(self.cols*0.2) : self.cols]
        self.rows, self.cols, b = self.cv_image_cortada.shape
        self.hsv = cv2.cvtColor(self.cv_image_cortada,cv2.COLOR_BGR2HSV)
        lowerblaranja = np.array([0, 110, 230])
        upperblaranja = np.array([26, 160, 255])
        mask_laranja = cv2.inRange(self.hsv, lowerblaranja, upperblaranja)    

        soma_lar = 0
        self.encontrou_lar = False
        for i in range (self.rows):
            for j in range (self.cols):
                if  mask_laranja[i, j] >= 200:
                    soma_lar += 1

        if (soma_lar > MIN_LAR):
            self.soma += 1

        if(self.soma > 2):
            self.encontrou_lar = True
            rospy.loginfo("TUBO DETECTADO")
            
    
        cv2.imshow('mask_laranja', mask_laranja)
        cv2.waitKey(0)
    
    def scan(self, number):
        if number == 0:
            cord_x = 50
            cord_y = -5
            self.mav.set_position(50, -5, 50, hdg = 0)
            time.sleep(15)
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
        if number == 1:
            cord_x = -45
            cord_y = -5
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
            
            one_pixel_in_meters = 0.28     #0.224691382978
            image_center_x = self.image_pixel_width/2
            image_center_y = self.image_pixel_height/2

            dif_x = cx - image_center_x
            dif_y = cy - image_center_y

            meters_y = dif_x * one_pixel_in_meters
            meters_x = dif_y * one_pixel_in_meters

            cord_base_x = cord_x - meters_x
            cord_base_y = cord_y - meters_y

            self.bases_moveis.append([cord_base_x,cord_base_y])
            print("Base localizada: " + str(cord_base_x) + " , " + str(cord_base_y))

        cv2.imshow('mask_BaseMovel', mask_BaseMovel)
        cv2.waitKey(3)

    def trajectory(self):
        self.scan(0)
        for base in self.bases_moveis:
            x,y = base
            rospy.loginfo("Indo para " + str(x) + " , " + str(y))
            self.mav.set_position(x,y,8)
            self.mav.set_position(x,y,-7.5)
            self.landing()
        self.mav.set_position(46,9,8)
        self.mav.set_position(46,9,-8.5)
        self.landing()
        self.mav.set_position(46,9,8)
        self.MAV.set_position(-19,-21,2)
        self.landing()
        self.scan(1)


    def landing(self):
        for i in range(40):
            self.cv_control_publisher.publish(Bool(True))
            self.rate.sleep()
        while self.land == 0:
            self.rate.sleep()
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=4):
            self.rate.sleep()
        self.mav.arm()
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=4):
            self.rate.sleep()
        self.mav.takeoff()
        self.land = 0


if __name__ == "__main__":
    rospy.init_node("fase1")
    mav = MRS_MAV("uav1")
    missao = fase1(mav)
    missao.trajectory()
