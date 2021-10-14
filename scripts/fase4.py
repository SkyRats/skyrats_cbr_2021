#!/usr/bin/env python3
import cv2
import time
import rospy
import mavros_msgs
import numpy as np
from pickle import FALSE
from MRS_MAV import MRS_MAV
from traceback import print_tb
from pyzbar.pyzbar import decode
from rospy.topics import Publisher
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
import detection_with_ros as Detection
from skyrats_cbr_2021.msg import H_info
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image, Range


TOL=0.2
TOL_BASE = 5
VEL_CERTO_X = 0.2
VEL_CERTO_Y = 0.2

class fase4:

    def __init__(self,mav):
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.drone = mav
        self.rate = rospy.Rate(60)
        self.last_time = time.time()
        self.bridge_object = CvBridge()

        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10) 
        
        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)


        # Attributes
        self.velocity = Vector3()
        self.giveup = False
        self.is_lost = True
        self.qrdata = None
        self.first_lost = 0
        self.land = 0
        self.decodificou = 0
        self.delay = 0
        self.flag = 0
        self.qr_encontrado = 0  
        self.perto_qr = 0


        self.setpoint_x = 480/2  # y size
        self.setpoint_y = 752/2  # x



    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        if self.perto_qr == 1:
            self.decode_qr()

    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()

    def detector(self):
        self.rows, self.cols, a = self.cv_image.shape
        self.hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
        lowerbbranco = np.array([0, 0, 250])
        upperbbranco = np.array([5, 5, 255])
        mask_branco = cv2.inRange(self.hsv, lowerbbranco, upperbbranco)    

        contours, hierarchy = cv2.findContours(mask_branco, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 1:
            for cnt in contours:
                M = cv2.moments(cnt)
                try: 
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.qrcode_center_x = cx
                    self.qrcode_center_y = cy

                    x,y,w,h = cv2.boundingRect(cnt)
                    max_w = w
                    max_h = h

                    self.qrcode_area_z = (max_w * max_h) / ( self.rows *  self.cols)
                    print(self.qrcode_area_z)
                    self.last_time = time.time()
                    self.qr_encontrado = 1

                except ZeroDivisionError:
                    pass                       
            cv2.drawContours(mask_branco, contours, -1, (0, 255, 0), 3)
            cv2.imshow('QR',mask_branco)
            cv2.waitKey(3)
            



    def decode_qr(self):
        #barcode_antigo = None
        self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #img_bw = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)[1] #thresh = 40


        qr_result = decode(self.gray)
        for barcode in qr_result:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # the barcode data is a bytes object so if we want to draw it
            # on our output image we need to convert it to a string first
            self.barcodeData = barcode.data.decode("utf-8")
            rospy.loginfo("QRCODE: " + str(self.barcodeData))
            self.decodificou = 1
            

            

            '''if(self.barcodeData != barcode_antigo):
                barcode_antigo = self.barcodeData
                flag = 0'''

            '''if(flag == 0):
                rospy.loginfo("QRCODE: " + str(self.barcodeData))
                flag = 1'''


    def boat_centralize(self):
        self.delay = time.time() - self.last_time
        self.is_lost = self.delay > 3
        if not self.is_lost:
            if self.detection.area_ratio < 0.25:  # Drone ainda esta longe do H
                if(self.flag == 0):
                    rospy.loginfo("Controle PID")
                    self.flag = 1
                
                erro_x = self.detection.center_y - self.setpoint_x 
                erro_y = self.detection.center_x - self.setpoint_y 

                p = 0.01
                    
                self.velocity.x= -erro_x * p
                self.velocity.y = -erro_y * p
                if self.velocity.x > 1:
                    self.velocity.x =1
                if self.velocity.x < -1:
                    self.velocity.x =-1
                if self.velocity.y > 1:
                    self.velocity.y =1
                if self.velocity.y < -1:
                    self.velocity.y =-1

                if(abs(self.velocity.x) < VEL_CERTO_X and abs(self.velocity.y) < VEL_CERTO_Y):
                    self.velocity.z = -1
                else:
                    self.velocity.z = 0
                

                for b in range(10):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()

            else:
                self.velocity.x = self.velocity.y = self.velocity.z = 0
                for j in range(20):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()
                if (self.flag == 1):
                    rospy.loginfo("Centralizando no QR")
                    self.flag = 0
                    

                self.land = 1
        else:
            rospy.loginfo("Base fora do campo de visao")
        self.rate.sleep()

    def qr_centralize(self):
        self.perto_qr = 1
        self.detector()
        self.delay = time.time() - self.last_time
        self.is_lost = self.delay > 3
        if self.qr_encontrado:
            if not self.is_lost:
                if self.qrcode_area_z < 0.01:  # Drone ainda esta longe do H
                    if(self.flag == 0):
                        rospy.loginfo("Controle PID")
                        self.flag = 1
                    
                    #if self.qrcode_area_z < 0.02:

                    erro_x = self.qrcode_center_y - self.setpoint_x 
                    erro_y = self.qrcode_center_x- self.setpoint_y 

                    p = 0.007
                        
                    self.velocity.x= -erro_x * p
                    self.velocity.y = -erro_y * p
                    if self.velocity.x > 1:
                        self.velocity.x =1
                    if self.velocity.x < -1:
                        self.velocity.x =-1
                    if self.velocity.y > 1:
                        self.velocity.y =1
                    if self.velocity.y < -1:
                        self.velocity.y =-1

                    if(abs(self.velocity.x) < VEL_CERTO_X and abs(self.velocity.y) < VEL_CERTO_Y):
                        self.velocity.z = -1
                    else:
                        self.velocity.z = 0
                    

                    for b in range(10):
                        self.vel_publisher.publish(self.velocity)
                        self.rate.sleep()

                else:
                    self.velocity.x = self.velocity.y = self.velocity.z = 0
                    for j in range(20):
                        self.vel_publisher.publish(self.velocity)
                        self.rate.sleep()
                    if (self.flag == 1):
                        rospy.loginfo("Tentando detectar")
                        self.flag = 0
                        
        else:
            rospy.loginfo("QR fora do campo de visao")
        self.rate.sleep()

    def run(self):
        ''' #teste
        rospy.loginfo("Indo para o teste!")
        self.go_to_fix("offshore1")
        self.boat_detect()
        #teste'''
        rospy.loginfo("Indo para base movel 1!")
        self.go_to_fix("movel1")
        self.boat_detect()
        self.drone.set_position(-30, 30, 4, hdg=1.57)
        rospy.loginfo("Indo para base movel 2!")
        self.go_to_fix("movel2")
        self.boat_detect()
        self.drone.set_position(60, 0, 4, hdg=1.57)
        rospy.loginfo("Indo para base movel 3!")
        self.go_to_fix("movel3")
        self.boat_detect()
        self.drone.set_position(30, -55, 4, hdg=1.57)


    def tempo(self,t):
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.rate.sleep()

    def boat_detect(self):        
        now = rospy.get_rostime()
        self.giveup = 0
        self.land = 0
        while self.land == 0 and not self.giveup :
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)):
               print("Desisto dessa base")
               self.giveup = 1
            for i in range(40):
                self.cv_control_publisher.publish(Bool(True))
                self.rate.sleep()
            self.boat_centralize()
        for i in range(40):
            self.cv_control_publisher.publish(Bool(False))
            self.rate.sleep()
        self.decodificou = 0
        now = rospy.get_rostime()
        self.giveup = 0
        while self.decodificou == 0 and not self.giveup :
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)):
               print("Desisto dessa base")
               self.giveup = 1
            for i in range(40):
                self.cv_control_publisher.publish(Bool(True))
                self.rate.sleep()
            self.qr_centralize()
        self.perto_qr = 0
        self.decodificou = 0
        '''now = rospy.get_rostime()
        self.giveup = 0
        rospy.loginfo("Starting QR Detection")
        while self.decodificou == 0 and not self.giveup:
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)):
               print("Desisto dessa base")
               self.giveup = 1
            self.decode_qr()
        self.decodificou = 0'''

    def go_to_fix(self, base):
        if base == "offshore1":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(-19.10, -21.1, 5, hdg= 1.57)
            #self.drone.altitude_estimator("HEIGHT")
            #self.drone.set_position(-19.10, -21.1, 0.55)

        if base == "offshore2":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(-50, -21, 4, hdg=1.57)
            self.drone.set_position(-53.7, -35.2, 4, hdg=1.57)
            self.drone.altitude_estimator("HEIGHT")
            self.drone.set_position(-53.7, -35.2, 0.55)

        if base == "movel3":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(30, -55, 4, hdg=1.57)
            self.drone.set_position(30, -55, -6, hdg=1.57)
        
        if base == "movel2":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(60, 0, 4, hdg=1.57)
            self.drone.set_position(60, 0, -6, hdg=1.57)
        
        if base == "movel1":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(-30, 30, 4, hdg=1.57)
            self.drone.set_position(-30, 30, -6, hdg=1.57)

if __name__ == "__main__":
    rospy.init_node("fase4")
    drone = MRS_MAV("uav1")
    a = fase4(drone)
    a.run()

