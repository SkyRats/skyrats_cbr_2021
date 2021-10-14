#!/usr/bin/env python3
from traceback import print_tb
import rospy
import mavros_msgs
from std_msgs.msg import Bool, String
import detection_with_ros as Detection
from MRS_MAV import MRS_MAV
from geometry_msgs.msg import Vector3
import numpy as np

TOL=0.2
TOL_BASE = 5

class fase4:

    def __init__(self,mav):
        self.rate = rospy.Rate(60)
        self.drone = MRS_MAV("uav1")
        self.running_pub = rospy.Publisher("/qrdetection/set_running_state", Bool, queue_size=10)  # sends data that tells when to start and stop detection
        self.detection_sub = rospy.Subscriber("/qrdetection/detection_status", Bool, self.detection_callback) # receives data about when the QR detection is sucessful or not
        self.data_sub = rospy.Subscriber("/qrdetection/qr_data", String, self.qrdata_callback) # receives data read from the QR Code
        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.done_pub = rospy.Publisher('/precision_landing/done', Bool, queue_size=10)
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10) 
        self.give_up_sub = rospy.Subscriber("/precision_landing/giveup", Bool, self.give_up_callback)
        self.velocity = Vector3()
        self.bases = [["A",0,0,0],["B",0,0,0],["C",30,40,-10],["D",-54,-34,-0.5],["E", -19.5,-20,3.5]] 
        self.bases_visitadas = []
        self.giveup = False
        self.alto = 0
        self.cont = 0
        self.qrdata = None
        self.qrdetection = False

    def give_up_callback(self,data):
        self.giveup = data.data

    def detection_callback(self,bool):
        self.qrdetection = bool.data

    def qrdata_callback(self,data):
        self.qrdata = data.data

    def run(self):
        self.drone.set_position(-19.5, -21, 8,1.57)
        self.drone.set_position(-19.5, -21, 5,1.57)
        for lista in self.bases:
            if abs(lista[1] + 19.5) < TOL_BASE and abs(lista[2] + 21) < TOL_BASE:
                self.bases_visitadas.append(str(lista[0]))
    
        now = rospy.get_rostime()
        while(self.qrdetection == False and not self.giveup):
            if rospy.get_rostime() - now > rospy.Duration(secs=50):
                    print("Desisto dessa base")
                    self.giveup = 1
            for i in range(40):
                #running_pub.publish(Bool(True))
                self.cv_control_publisher.publish(Bool(True))
                self.rate.sleep()
        for i in range(60):
            self.done_pub.publish(Bool(True))
            self.running_pub.publish(Bool(False))
            self.cv_control_publisher.publish(Bool(False))
            self.rate.sleep()
        self.vel0()
        self.tempo(2)
        self.drone.arm()
        self.tempo(2)
        self.drone.takeoff()
        rospy.loginfo("Bases visitadas: " + str(self.bases_visitadas))

        while len(self.bases_visitadas) < 5:
            if self.qrdata in self.bases_visitadas:
                for lista in self.bases:
                    if lista[0] not in self.bases_visitadas:
                        x,y,z = self.coordenadas_base(lista[0])
            else:
                x,y,z = self.coordenadas_base(qrdata)

            if (x < -19.5 and y < 21) or self.alto == 1:
                self.drone.set_position(self.drone.controller_data.position.x,self.drone.controller_data.position.y,30,1.57)
                self.drone.set_position(x,y,30,1.57)
                if self.alto == 1:
                    self.alto = 0
                else:
                    self.alto = 1
            else:
                self.drone.set_position(self.drone.controller_data.position.x,self.drone.controller_data.position.y,9,1.57)
                self.drone.set_position(x,y,9,1.57)


            self.drone.set_position(x,y,z+3,1.57)
            now = rospy.get_rostime()
                            
            while(self.qrdetection == False and not self.giveup):
                if rospy.get_rostime() - now > rospy.Duration(secs=50):
                    print("Desisto dessa base")
                    self.giveup = 1
                for i in range(40):
                    #running_pub.publish(Bool(True))
                    self.cv_control_publisher.publish(Bool(True))
                    self.rate.sleep()
            for i in range(60):
                self.done_pub.publish(Bool(True))
                self.running_pub.publish(Bool(False))
                self.cv_control_publisher.publish(Bool(False))
                self.rate.sleep()
            for lista in self.bases:
                if abs(lista[1] - x) < TOL_BASE and abs(lista[2] - y) < TOL_BASE:
                    self.bases_visitadas.append(str(lista[0]))
            self.vel0()
            self.tempo(2)
            self.drone.arm()
            self.tempo(2)
            self.drone.takeoff()
            rospy.loginfo("Bases visitadas: " + str(self.bases_visitadas))

        if self.alto == 1:
            self.drone.set_position(self.drone.controller_data.position.x,self.drone.controller_data.position.y,30,1.57)
            self.drone.set_position(10,90,30,1.57)
                
        else:
            self.drone.set_position(self.drone.controller_data.position.x,self.drone.controller_data.position.y,9,1.57)
            self.drone.set_position(10,90,9,1.57)
        
        self.drone.set_position(10,90,2,1.57)
        self.drone.land()
        self.tempo(2)
        self.drone.disarm()

    def coordenadas_base(self,letra):
        for lista in self.bases:
            if lista[0] == letra:
                return lista[1],lista[2],lista[3]

    def tempo(self,t):
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.rate.sleep()

    def vel0(self):
        self.velocity.x = self.velocity.y = self.velocity.z = 0
        for j in range(40):
            self.vel_publisher.publish(self.velocity)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("qr_mission")
    drone = MRS_MAV("uav1")
    a = fase4(drone)
    a.run()

