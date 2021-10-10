#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from rospy.core import is_shutdown
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge,CvBridgeError
import time
from std_msgs.msg import Bool


MIN_LAR = 10000

########## DETECTA LARANJA  ################################################
class pipeline_detector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.cv_state = rospy.Subscriber("/pipline_detector/set_running_state", Bool, self.cv_callback)
        self.bridge_object = CvBridge()
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.encontrou_lar = 0
        self.cv = 0
        self.soma = 0


    def cv_callback(self, data):
        self.cv = data.data

    def camera_callback(self, data):
        cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.rows, self.cols, a = cv_image.shape
        self.cv_image = cv_image[0: int(self.rows - (self.rows*0.2)) , int(self.cols*0.2) : self.cols]
        self.rows, self.cols, b = self.cv_image.shape
        self.hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)

    def detector(self):
        lowerblaranja = np.array([0, 110, 230])
        upperblaranja = np.array([26, 160, 255])
        mask_laranja = cv2.inRange(self.hsv, lowerblaranja, upperblaranja)    

        soma_lar = 0
        self.encontrou_lar = False
        for i in range (self.rows):
            for j in range (self.cols):
                if  mask_laranja[i, j] >= 200:
                    soma_lar += 1

        print("Laranjas: ")
        print(soma_lar)
        if (soma_lar > MIN_LAR):
            self.soma += 1

        if(self.soma > 2):
            self.encontrou_lar = True
            rospy.loginfo("TUBO DETECTADO")
            
    
        cv2.imshow('mask_laranja', mask_laranja)
        cv2.waitKey(3)

    def run(self):
        while(not rospy.is_shutdown() and self.encontrou_lar == 0):
            if self.cv:
                self.detector()

if __name__ == '__main__':
    rospy.init_node('pipeline_detector_node')
    a = pipeline_detector()
    a.run()