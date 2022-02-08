#!/usr/bin/env python3
import cv2
import time
import rospy
import numpy as np
from pickle import FALSE
from MRS_MAV import MRS_MAV
from traceback import print_tb
from pyzbar.pyzbar import decode
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from skyrats_cbr_2021.msg import H_info
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class fase4:

    def __init__(self,mav):
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.drone = mav
        self.rate = rospy.Rate(60)
        self.last_time = time.time()
        self.bridge_object = CvBridge()

        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)


        # Attributes
        self.qrdata = None
        self.land = 0
        self.decodificou = 0
        self.delay = 0
        self.flag = 0
        self.qr_encontrado = 0  
        self.perto_qr = 0
        self.barcode_antigo = None

        self.repetido = 0


    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        if self.perto_qr == 1:
            self.decode_qr()

    def decode_qr(self):    #Decodifica o qrcode
        self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #img_bw = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)[1] #thresh = 40


        qr_result = decode(self.gray)
        for barcode in qr_result:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # the barcode data is a bytes object so if we want to draw it
            # on our output image we need to convert it to a string first
            self.barcodeData = barcode.data.decode("utf-8")
            self.decodificou = 1            

            if(self.barcodeData != self.barcode_antigo): #Evita que o dado do qrcode seja printado diversas vezes 
                self.barcode_antigo = self.barcodeData
                self.repetido = 0

            if(self.repetido == 0):
                rospy.loginfo("QRCODE: " + str(self.barcodeData))   
                self.repetido = 1


   
    def run(self):
        rospy.loginfo("Indo para base offshore 1")
        self.go_to_fix("offshore1")
        self.fix_detect()
        rospy.loginfo("Indo para base offshore 2")
        self.go_to_fix("offshore2")
        self.fix_detect()
        self.drone.set_position(self.drone.controller_data.position.x, self.drone.controller_data.position.y, 9,1.57)
        self.drone.set_position(10,90,9,1.57)
        self.drone.altitude_estimator("HEIGHT")
        self.drone.set_position(10,90,0.55,1.57)
        self.drone.land()    
        self.tempo(4)
        self.drone.disarm()




    def tempo(self,t):
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.rate.sleep()


    def fix_detect(self): #Tenta decodificar o qrcode por 60 seg
        self.decodificou = 0
        self.giveup = 0
        now = rospy.get_rostime()
        while self.decodificou == 0 and not self.giveup :
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)):
               print("Desisto dessa base")
               self.giveup = 1
            self.decode_qr()
        self.decodificou = 0
        self.drone.altitude_estimator("BARO")


   
    def go_to_fix(self, base):
        if base == "offshore1":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(-19.10, -21.1, 7, hdg= 1.57)
            self.drone.altitude_estimator("HEIGHT")
            self.drone.set_position(-19.10, -21.1, 0.55)

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



    def vel0(self):
        self.velocity.x = self.velocity.y = self.velocity.z = 0
        for j in range(20):
            self.vel_publisher.publish(self.velocity)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fase4_fixa")
    drone = MRS_MAV("uav1")
    a = fase4(drone)
    a.run()

