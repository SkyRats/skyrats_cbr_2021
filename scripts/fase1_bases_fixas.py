#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from MRS_MAV import MRS_MAV
from std_msgs.msg import Bool
from pickle import FALSE
import time
from sensor_msgs.msg import Range

MIN_LAR = 1200


class fase1:
    def __init__(self,mav):
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.lidar_sub = rospy.Subscriber("/uav1/garmin/range", Range, self.lidar_callback)
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.mav = mav
        self.bases_visitadas = 0        
        self.encontrou_lar = 0
        self.soma = 0
        self.rate = rospy.Rate(60)
        self.last_time = time.time()


        # Cam Params
        self.image_pixel_width = 752
        self.image_pixel_height = 480        

        self.setpoint_x = self.image_pixel_height/2  # y size
        self.setpoint_y = self.image_pixel_width/2  # x

    def lidar_callback(self,data):
        self.lidar_range = data.range

    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")


    def tubo(self):
        if self.encontrou_lar == False:
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
                

    #ALTURA DA TRAJETORIA
    def trajectory(self): 
        self.mav.altitude_estimator("BARO")
        self.mav.set_position(-49.6, -24.7, 3, relative_to_drone=False)
        rospy.loginfo("Indo para offshore2")
        self.go_to_fix("offshore2")
        self.landing()
        rospy.loginfo("Procurando Tubo...")
        self.mav.set_position(-50, -36, 4,1.57)
        for i in range(40):
            self.tubo()
            self.rate.sleep()
        self.mav.set_position(-50, -21, 4,1.57)
        rospy.loginfo("Indo para offshore1")
        self.go_to_fix("offshore1")
        self.landing()
        rospy.loginfo("Indo para o pier")
        self.go_to_fix("pier")
        self.landing()
        self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 9,1.57)
        self.mav.set_position(10,90,9,1.57)
        self.mav.altitude_estimator("HEIGHT")
        self.mav.set_position(10, 90, 0.55,1.57)
        self.mav.land()    
        self.time(4)
        self.mav.disarm()
           

    def landing(self):    
        self.mav.land()    
        self.time(5)
        self.mav.disarm()
        self.bases_visitadas += 1
        rospy.loginfo("N bases visitadas: " + str(self.bases_visitadas))
        self.time(5)
        self.mav.arm()
        self.time(5)
        self.mav.takeoff()
        self.time(6)
        self.mav.altitude_estimator("BARO")

    
    def time(self, t):
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.rate.sleep()


    def go_to_fix(self, base):
        if base == "pier":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(45.4, 10, 4,1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(45.4, 10, 0.55,1.57)

        if base == "offshore1":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(-19.10, -21.1, 4, hdg= 1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(-19.10, -21.1, 0.55,1.57)

        if base == "offshore2":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(-53.7, -35.2, 4, hdg=1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(-53.7, -35.2, 0.55,1.57)

if __name__ == "__main__":
    rospy.init_node("fase1")
    mav = MRS_MAV("uav1")
    missao = fase1(mav)
    missao.trajectory()
