#!/usr/bin/env python3

import time
import rospy
import numpy as np
from re import X
from MRS_MAV import MRS_MAV
from rospy.core import is_shutdown
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

TOL = 1

class TrajectoryTest():
    def __init__(self, MAV):
        self.rate = rospy.Rate(60)
        self.MAV = MAV

        self.Lista_das_bases = []
        self.parte_missao = 0
        self.altura = -6        #Altura da trajetoria
        self.stop = 0
        self.vel = 4

        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.cv_pipeline_publisher = rospy.Publisher("/pipline_detector/set_running_state", Bool, queue_size=10)
        self.last_land_publisher = rospy.Publisher("/precision_land/last_land", Bool, queue_size=10)

        self.stop_sub = rospy.Subscriber("/stop_trajectory", Bool, self.stop_callback)
        self.base_sub = rospy.Subscriber("/base_position", Vector3, self.base_pos_callback)

    def base_pos_callback(self, data):  #Recebe posicao das bases encontradas
        self.base_pos_x = data.x
        self.base_pos_y = data.y
        if [data.x,data.y] not in self.Lista_das_bases: 
            self.Lista_das_bases.append([self.base_pos_x,self.base_pos_y])

    def stop_callback(self, data):      #Recebe ordem para parar a trajetoria
        self.stop = data.data

    def run(self):
        goal_x = initial_x = 30
        initial_y = 40
        largura = 13
        curva = 0
        loop = 0
        goal_y = 9
        self.MAV.set_position(initial_x,initial_y,5)  #Ponto de partida
        rospy.loginfo("Ponto Inicial Da Trajetoria")

        while(not rospy.is_shutdown() and self.parte_missao <13):
            for i in range(10):
                if(self.parte_missao == 4 or self.parte_missao == 8 or self.parte_missao == 7):
                    self.cv_control_publisher.publish(Bool(False))   
                else:
                    self.cv_control_publisher.publish(Bool(self.verificar_area()))   #Liga a deteccao da cruz
                
                if(self.parte_missao == 7):
                    self.cv_pipeline_publisher.publish(Bool(True))
                else:
                    self.cv_pipeline_publisher.publish(Bool(False))

                self.rate.sleep()

            if(self.stop == False):

                if(self.parte_missao == 0):
                    rospy.loginfo("Descendo para altura de varredura")
                    self.MAV.set_position(initial_x, initial_y, self.altura)
                    rospy.loginfo("Varrendo em direcao a base 1")
                    self.MAV.set_position(initial_x, goal_y, self.altura)
                    if (self.MAV.controller_data.position.y - goal_y < TOL):
                        rospy.loginfo("Estamos perto da base 1")
                        self.parte_missao = 1


                if(self.parte_missao == 1):
                    self.MAV.set_position(45.7,9.8,-8.5)
                    rospy.loginfo("Estamos em cima da base 1")
                    self.parte_missao = 2
                    

                elif(self.parte_missao == 2):
                    self.MAV.set_position(46,9,self.altura)
                    rospy.loginfo("Voltando para varredura")
                    self.MAV.set_position(30,10,self.altura)
                    rospy.loginfo("Iniciando ZigZag")
                    self.parte_missao = 3


                elif(self.parte_missao == 3):
                    if (curva == 0):
                        initial_y = 10
                        goal_y = -30 #-20
                        if(loop%2 == 1):
                            backup_y = goal_y
                            goal_y = initial_y 
                            initial_y = backup_y
                            self.MAV.set_position(goal_x, goal_y, self.altura)

                            if(self.MAV.controller_data.position.y - goal_y < TOL):
                                loop = loop + 1
                                goal_x -= largura
                                curva = 1

                        else:
                            self.MAV.set_position(goal_x, goal_y, self.altura)
                            if(self.MAV.controller_data.position.y - goal_y < TOL):
                                loop += 1
                                goal_x -= largura
                                curva = 1

                    if(curva == 1):
                        self.MAV.set_position(goal_x, goal_y, self.altura)

                        if(self.MAV.controller_data.position.x - goal_x < TOL):
                            curva = 0

                    if(loop == 3):
                        self.parte_missao = 4
                        rospy.loginfo("Fim zigzag")

                

                elif (self.parte_missao == 4):
                    self.MAV.set_position(goal_x,goal_y,5)
                    self.MAV.set_position(-19,-21,5)
                    self.parte_missao = 5    


                elif self.parte_missao == 5:
                    self.MAV.set_position(-19,-21,2)
                    self.parte_missao = 6

                elif self.parte_missao == 6:
                    rospy.loginfo("Indo para o Tubo")
                    self.MAV.set_position(-50,-21,5)
                    self.MAV.set_position(-55.5,-26, 5)
                    self.parte_missao = 7

                
                elif self.parte_missao == 7:
                    self.MAV.set_position(-50,-36,5)
                    self.parte_missao = 8
                

                elif self.parte_missao == 8:
                    rospy.loginfo("Indo para base 3")
                    self.MAV.set_position(-55.5,-36,5)
                    self.parte_missao = 9


                elif self.parte_missao == 9:
                    self.MAV.set_position(-55.5,-36,-2)
                    self.parte_missao = 10


                elif self.parte_missao == 10:
                    self.MAV.set_position(-50,-21,5)
                    self.parte_missao = 11


                elif self.parte_missao == 11:
                    self.MAV.set_position(-50,-10,self.altura)
                    self.MAV.set_position(-50, 40,self.altura)
                    self.MAV.set_position(-50, 40, 6)
                    rospy.loginfo("RTL")
                    for i in range (40):
                        self.last_land_publisher.publish(Bool(True))  
                        self.rate.sleep()
                    self.MAV.set_position(10,90,6)
                    self.parte_missao = 12


                elif self.parte_missao == 12:
                    self.MAV.set_position(10,90,2)
                    self.parte_missao = 13
            

            else:
                self.rate.sleep()


            self.rate.sleep()

    def verificar_area(self): 
        for [x,y] in self.Lista_das_bases:
            if( np.power((self.MAV.controller_data.position.x - x),2) + np.power((self.MAV.controller_data.position.y - y),2) <= 144):
                return False
        return True

if __name__ == "__main__":
    rospy.init_node('longo')
    drone = MRS_MAV("uav1")
    c = TrajectoryTest(drone)
    c.run()