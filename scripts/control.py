#!/usr/bin/env python3
from pickle import FALSE
import rospy
import time
import numpy as np

from MRS_MAV import MRS_MAV
from skyrats_cbr_2021.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from simple_pid import PID
from mrs_msgs.msg import PositionCommand
from sensor_msgs.msg import Range

VEL_CERTO_X = 0.3
VEL_CERTO_Y = 0.2

class PrecisionLanding():
    def __init__(self, MAV):
        # ROS setup
        self.rate = rospy.Rate(60)
        self.MAV = MAV

        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.last_time = time.time()
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10)
        self.base_publisher = rospy.Publisher("/base_position", Vector3, queue_size=10)
        self.land_pub = rospy.Publisher("/precision_land/land", Bool, queue_size=10)
        self.lidar_sub = rospy.Subscriber("/uav1/garmin/range", Range, self.lidar_callback)
        self.giveup_sub = rospy.Subscriber("/precision_landing/giveup", Bool, self.giveup_callback)
        self.achou_pub = rospy.Publisher("/precision_landing/achou", Bool, queue_size=10)
        self.cv_control_publisher = rospy.Subscriber("/precision_landing/set_running_state", Bool, self.runnin_callback)

        # Cam Params
        self.image_pixel_width = 752
        self.image_pixel_height = -480

        # Attributes
        self.giveup = 0
        self.delay = 0
        self.is_lost = True
        self.first_lost = 0
        self.flag = 0
        self.done = 0
        self.first = True
        self.velocity = Vector3()
        self.running = 0
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
        self.pid_z.setpoint = 0.7 #Podemos mudar para um lidar (fazer um filtro)
        self.pid_w.setpoint = 0  # orientation

        # Limitacao da saida
        self.pid_x.output_limits = self.pid_y.output_limits = (-1, 1)
        self.pid_z.output_limits = (-1.5, 1.5)

    def runnin_callback(self,data):
        self.running =data.data

    def giveup_callback(self,data):
        self.giveup = data.data

    def lidar_callback(self,data):
        self.lidar_range = data.range
        
    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
        self.first_detection = 1

    def precision_land(self):
        while not rospy.is_shutdown():
            self.delay = time.time() - self.last_time
            self.is_lost = self.delay > 3
            if self.first_detection == 1:
                if self.detection.center_x == -1:
                    self.is_lost = 1
            
            if not self.is_lost and self.first_detection == 1 and self.giveup == 0:
                if self.detection.area_ratio < 0.45:  # Drone ainda esta longe do H
                    if(self.flag == 0):
                        rospy.loginfo("Controle PID")
                        self.flag = 1
                    
                    self.velocity.x= self.pid_x(-self.detection.center_y)
                    self.velocity.y = self.pid_y(self.detection.center_x)
                    if(abs(self.velocity.x) < VEL_CERTO_X and abs(self.velocity.y) < VEL_CERTO_Y):
                        self.velocity.z = self.pid_z(self.detection.area_ratio)
                    else:
                        self.velocity.z = 0

                    print("Vel_x = " + str(self.velocity.x))
                    print("Vel_y = " + str(self.velocity.y))
                    print("Vel_z = " + str(self.velocity.z))

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
                        for i in range(40):
                            self.achou_pub.publish(Bool(True))
                            self.rate.sleep()
                        self.flag = 0
                    
                    now = rospy.get_rostime()
                    while not rospy.get_rostime() - now > rospy.Duration(secs=1):
                        self.rate.sleep()
                    #self.MAV.altitude_estimator("HEIGHT")
                    self.MAV.set_position(0.2, 0,0,0, relative_to_drone=True)
                    self.MAV.land()
                    while self.lidar_range > 0.25:
                        pass
                    self.MAV.disarm()
                    for i in range(40):
                        self.land_pub.publish(Bool(True))
                        self.rate.sleep()
            elif self.running == 1:
                self.MAV.set_position(0.15,0,0,0,relative_to_drone=True)
                print("de ladin")

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('control')
    drone = MRS_MAV("uav1")
    c = PrecisionLanding(drone)
    c.precision_land()
