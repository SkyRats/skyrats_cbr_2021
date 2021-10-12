#!/usr/bin/env python3
from pickle import FALSE
import rospy
import time
import numpy as np

from MRS_MAV import MRS_MAV
from skyrats_cbr_2021.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Bool
from simple_pid import PID
from mrs_msgs.msg import PositionCommand
VEL_CERTO = 0.09

class PrecisionLanding():
    def __init__(self, MAV):
        # ROS setup
        self.rate = rospy.Rate(60)
        self.MAV = MAV

        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.done_sub = rospy.Subscriber('/precision_landing/done', Bool, self.done_callback)

        #self.qrcode_center_sub = rospy.Subscriber("/qrcode_finder/center", Point, self.qrcode_center_callback)
        self.last_time = time.time()
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10) 
        self.give_up_publisher = rospy.Publisher("/precision_landing/giveup", Bool, queue_size=10)

        #self.last_land_sub = rospy.Subscriber("/precision_land/last_land", Bool, self.last_land_callback)
        self.running_pub = rospy.Publisher("/qrdetection/set_running_state", Bool, queue_size=10)  # sends data that tells when to start and stop detection

        # Cam Params
        self.image_pixel_width = 752
        self.image_pixel_height = -480

        # Attributes
        self.delay = 0
        self.is_lost = True
        self.first_lost = 0
        self.flag = 0
        self.done = 0
        self.first = True
        self.velocity = Vector3()
        self.first_detection = 0
        talz = 15 #segundos
        kpz = 0.5
        # PIDs
        # Parametros Proporcional,Integrativo e Derivativo
        self.pid_x = PID(-0.005, 0, -0)
        self.pid_y = PID(0.005, 0, 0)
        # Negative parameters (CV's -y -> Frame's +z)
        self.pid_z = PID(-kpz, -kpz/talz, 0)
        self.pid_w = PID(0, 0, 0)  # Orientation

        self.pid_x.setpoint = self.image_pixel_height/2  # y size
        self.pid_y.setpoint = self.image_pixel_width/2  # x
        self.pid_z.setpoint = 0.7 #Podemos mudar para um lidar (fazer um filtro)
        self.pid_w.setpoint = 0  # orientation

        # Limitacao da saida
        self.pid_x.output_limits = self.pid_y.output_limits = (-1, 1)
        self.pid_z.output_limits = (-0.5, 0.5)


    '''def qrcode_center_callback(self, data):
        self.center_x = data.x
        self.center_y = data.y
        self.area_ratio = data.z
        #print("Centro x: " + str(self.center_x ))
        #print("Centro y: " + str(self.center_y ))
        #print("Area ratio: " + str(self.area_ratio))

        self.last_time = time.time()
        self.first_detection = 1'''

    def done_callback(self, data):
        self.done = data.data

    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
        self.first_detection = 1 

    def precision_land(self):

        while not rospy.is_shutdown():
            self.done = 0
            flag =0 
            while(self.done == 0):

                self.delay = time.time() - self.last_time
                if self.first:
                    self.is_lost = True
                else:
                    self.is_lost = self.delay > 3
                if self.first_detection == 1:
                    if self.detection.center_x == -1:
                        self.is_lost = 1
                
                if not self.is_lost and self.first_detection == 1:
                    print("Area: " + str(self.detection.area_ratio))
                    if self.done == 0:
                        if self.detection.area_ratio < 0.35:  # Drone ainda esta longe do H
                            if(flag == 0):
                                flag = 1
                            
                            if(self.detection.area_ratio > 0.15):
                                for i in range(20):
                                    self.running_pub.publish(Bool(True))
                                    self.rate.sleep

                            self.velocity.x= self.pid_x(-self.detection.center_y)
                            self.velocity.y = self.pid_y(self.detection.center_x)
                            # PID z must have negative parameters
                            if(abs(self.velocity.x) < VEL_CERTO and abs(self.velocity.y) < VEL_CERTO):
                                self.velocity.z = self.pid_z(self.detection.area_ratio)
                            else:
                                self.velocity.z = 0
                        
                            for b in range(30):
                                self.vel_publisher.publish(self.velocity)
                                self.rate.sleep()

                        else:
                            self.velocity.x = self.velocity.y = self.velocity.z = 0
                            for j in range(40):
                                self.vel_publisher.publish(self.velocity)
                                self.rate.sleep()
                            self.MAV.land()
                            self.MAV.disarm()
                            now = rospy.get_rostime()
                            while not rospy.get_rostime() - now > rospy.Duration(secs=2):
                                self.rate.sleep()
                            for k in range(40):
                                self.give_up_publisher.publish(1)
                                self.rate.sleep()


                elif self.first:
                    rospy.loginfo("Iniciando...")
                    self.first = False

                elif self.done != 1 :  # Drone perdeu o H 
                    if flag == 1:        
                        lost = rospy.get_rostime()
                        self.first_lost = 1
                        self.velocity.x = self.velocity.y = self.velocity.z = 0
                        for l in range(20):
                            self.vel_publisher.publish(self.velocity)
                            self.rate.sleep()
                        rospy.loginfo("Perdi a cruz")
                        
                    flag = 0
                    if self.first_lost == 1:
                        if rospy.get_rostime() - lost > rospy.Duration(secs=10):
                            print("Desisto dessa base")       
                            for k in range(40):
                                self.give_up_publisher.publish(1)
                                self.rate.sleep() 
                            self.first_lost = 0
                    

                self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('control_qr')
    drone = MRS_MAV("uav1")
    c = PrecisionLanding(drone)
    c.precision_land()
