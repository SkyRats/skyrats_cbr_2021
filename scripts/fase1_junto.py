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
from skyrats_cbr_2021.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Vector3
from simple_pid import PID
from mrs_msgs.msg import PositionCommand

MIN_LAR = 1200
TOL_BASE = 8

VEL_CERTO_X = 0.3
VEL_CERTO_Y = 0.2

class fase1:
    def __init__(self,mav):
        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10)
        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.mav = mav
        self.bases_moveis_1 =[]
        self.bases_visitadas = 0        
        self.bases_moveis_0 = []
        self.encontrou_lar = 0
        self.land = 0
        self.soma = 0
        self.rate = rospy.Rate(60)
        self.last_time = time.time()


        # Cam Params
        self.image_pixel_width = 752
        self.image_pixel_height = 480

        # Attributes
        self.giveup = 0
        self.delay = 0
        self.is_lost = True
        self.first_lost = 0
        self.flag = 0
        self.done = 0
        self.first = True
        self.velocity = Vector3()
        self.first_detection = 0
        talz = 15 #segundos
        kpz = 1
        # PIDs
        # Parametros Proporcional,Integrativo e Derivativo
        self.pid_x = PID(-0.015, 0, -0)
        self.pid_y = PID(0.015, 0, 0)
        # Negative parameters (CV's -y -> Frame's +z)
        self.pid_z = PID(-kpz, -kpz/talz, 0)
        self.pid_w = PID(0, 0, 0)  # Orientation

        self.pid_x.setpoint = self.image_pixel_height/2  # y size
        self.pid_y.setpoint = self.image_pixel_width/2  # x
        self.pid_z.setpoint = 1 #Podemos mudar para um lidar (fazer um filtro)
        self.pid_w.setpoint = 0  # orientation

        # Limitacao da saida
        self.pid_x.output_limits = self.pid_y.output_limits = (-1, 1)
        self.pid_z.output_limits = (-1.5, 1.5)

    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
        self.first_detection = 1

    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")

    def precision_land(self):
            self.delay = time.time() - self.last_time
            self.is_lost = self.delay > 3
            if self.first_detection == 1:

                if not self.is_lost:
                    if self.detection.area_ratio < 0.40:  # Drone ainda esta longe do H
                        if(self.flag == 0):
                            rospy.loginfo("Controle PID")
                            self.flag = 1
                        
                        self.velocity.x= self.pid_x(-self.detection.center_y)
                        self.velocity.y = self.pid_y(self.detection.center_x)
                        if(abs(self.velocity.x) < VEL_CERTO_X and abs(self.velocity.y) < VEL_CERTO_Y):
                            self.velocity.z = self.pid_z(self.detection.area_ratio)
                        else:
                            self.velocity.z = 0
                        
                        print("Centerx " + str(self.detection.center_x))
                        print("Centery" + str(self.detection.center_y))                   
                        print("Velx: " + str(self.velocity.x))
                        print("Vely: " + str(self.velocity.y))
                        print("Area: " + str(self.detection.area_ratio))

                        for b in range(10):
                            self.vel_publisher.publish(self.velocity)
                            self.rate.sleep()

                    else:
                        self.velocity.x = self.velocity.y = self.velocity.z = 0
                        for j in range(20):
                            self.vel_publisher.publish(self.velocity)
                            self.rate.sleep()
                        if (self.flag == 1):
                            rospy.loginfo("Cruz encontrada!")
                            rospy.logwarn("Descendo...")
                            self.achou = 1
                            self.flag = 0
                        
                        self.time(1)
                        #self.MAV.altitude_estimator("HEIGHT")
                        self.mav.land()
                        self.time(3)
                        self.mav.disarm()
                        self.land = 1
                else:
                    
                    self.mav.set_position(0,-1,0,0,relative_to_drone=True)
                    print("de ladin")
                    self.rate.sleep()

            self.rate.sleep()

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
                
    
    def scan(self, number):
        if number == 0:
            cord_x = 50
            cord_y = -5
            self.mav.set_position(50, -5, 50, hdg = 0)
            rospy.loginfo("Iniciando Analise do terreno")
            self.time(8)
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
        if number == 1:
            cord_x = -45
            cord_y = -5
            self.mav.set_position(-45, -5, 50, hdg = 0)
            rospy.loginfo("Iniciando Analise do terreno")
            self.time(8)
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
            #cv2.imshow('cv_image',self.cv_image)
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

            if number == 0:
                self.bases_moveis_0.append([cord_base_x,cord_base_y])
            elif number == 1:
                self.bases_moveis_1.append([cord_base_x,cord_base_y])


            print("Base localizada: " + str(cord_base_x) + " , " + str(cord_base_y))

        '''cv2.imshow('mask_BaseMovel', mask_BaseMovel)
        cv2.waitKey(3)'''

    #ALTURA DA TRAJETORIA
    def trajectory(self):
        #self.mav.altitude_estimator("BARO")
        self.landing_control()
        self.scan(1)
        for base in self.bases_moveis_1:
            self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 28,1.57)
            x,y = base
            rospy.loginfo("Indo para " + str(x) + " , " + str(y))
            self.giveup = False
            self.mav.set_position(x,y,28,1.57)
            self.mav.set_position(x,y,-6,1.57)
            self.landing_control()

        if (self.mav.controller_data.position.x > -19.5 and self.mav.controller_data.position.y > 21):
            print("mav.controller_data.position: " + str(self.mav.controller_data.position.x)+ str(self.mav.controller_data.position.y))
            self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 28,1.57)
            self.mav.set_position(-53.7, -35.2, 28,1.57)
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

        if self.bases_visitadas < 6:
            self.scan(0)

            for base in self.bases_moveis_0:
                skip = 0
                x,y = base

                for lista in self.bases_moveis_1:
                    if abs(lista[0] - x) < TOL_BASE and abs(lista[1] - y) < TOL_BASE:
                        rospy.loginfo("Ja foi visitada a base " + str(x) + " , " +str(y))
                        skip = 1

                if skip == 0:
                    rospy.loginfo("Indo para " + str(x) + " , " + str(y))
                    self.giveup = False
                    self.mav.set_position(x,y,8,1.57)
                    self.mav.set_position(x,y,-7.5,1.57)
                    self.landing_control()
                    self.bases_visitadas += 1
                    rospy.loginfo("N bases visitadas: " + str(self.bases_visitadas))
        
        self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 9,1.57)
        self.mav.set_position(10,90,9,1.57)
        self.mav.land()    
        self.time(4)
        self.mav.disarm()



    def landing(self):    
        self.mav.land()    
        self.time(4)
        self.mav.disarm()
        self.bases_visitadas += 1
        rospy.loginfo("N bases visitadas: " + str(self.bases_visitadas))
        self.time(4)
        self.mav.arm()
        self.time(4)
        self.mav.takeoff()
        #self.time(6)
        #self.mav.altitude_estimator("BARO")


    def landing_control(self):        
        now = rospy.get_rostime()
        self.giveup = 0
        while self.land == 0 and not self.giveup :
            #if (rospy.get_rostime() - now > rospy.Duration(secs=60)):
            #   print("Desisto dessa base")
            #   self.giveup = 1
            for i in range(40):
                self.cv_control_publisher.publish(Bool(True))
                self.rate.sleep()
            self.precision_land()

        for i in range(40):
            self.cv_control_publisher.publish(Bool(False))
            self.rate.sleep()
        self.bases_visitadas += 1
        if self.giveup == 0:
            rospy.loginfo("N bases visitadas: " + str(self.bases_visitadas))
            self.time(6)
            self.mav.arm()
            self.time(4)
            self.mav.takeoff()



    def time(self, t):
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.rate.sleep()


    def go_to_fix(self, base):
        if base == "pier":
            #self.mav.altitude_estimator("BARO")
            self.mav.set_position(45.15, 10, 4,1.57)
            #self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(45.15, 10, -7,1.57)

        if base == "offshore1":
            #self.mav.altitude_estimator("BARO")
            self.mav.set_position(-19.10, -21.1, 4,1.57)
            #self.mav.altitude_estimator("HEIGHT")
            #self.mav.set_position(-19.10, -21.1, 0.55,1.57)

        if base == "offshore2":
            #self.mav.altitude_estimator("BARO")
            self.mav.set_position(-53.7, -35.2, 3,1.57)
            #self.mav.altitude_estimator("HEIGHT")
            #self.mav.set_position(-53.7, -35.2, 0.55,1.57)


if __name__ == "__main__":
    rospy.init_node("fase1")
    mav = MRS_MAV("uav1")
    missao = fase1(mav)
    missao.trajectory()
