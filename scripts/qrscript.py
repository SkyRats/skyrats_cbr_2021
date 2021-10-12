#!/usr/bin/env python3
from traceback import print_tb
import rospy
import mavros_msgs
from std_msgs.msg import Bool, String
import detection_with_ros as Detection
from MRS_MAV import MRS_MAV
from geometry_msgs.msg import Vector3
import numpy as np
qrdetection = False
qrdata = None
TOL=0.2
TOL_BASE = 5
global cont 
cont = 0

def give_up_callback(data):
    global giveup
    giveup = data.data

def detection_callback(bool):
    global qrdetection
    qrdetection = bool.data

def qrdata_callback(data):
    global qrdata
    qrdata = data.data

def run():
    rospy.init_node("qr_mission")
    global rate 
    rate = rospy.Rate(60)
    global qrdetection
    global qrdata
    global drone 
    drone = MRS_MAV("uav1")
    running_pub = rospy.Publisher("/qrdetection/set_running_state", Bool, queue_size=10)  # sends data that tells when to start and stop detection
    detection_sub = rospy.Subscriber("/qrdetection/detection_status", Bool, detection_callback) # receives data about when the QR detection is sucessful or not
    data_sub = rospy.Subscriber("/qrdetection/qr_data", String, qrdata_callback) # receives data read from the QR Code
    cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
    done_pub = rospy.Publisher('/precision_landing/done', Bool, queue_size=10)
    global vel_publisher 
    vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10) 
    give_up_sub = rospy.Subscriber("/precision_landing/giveup", Bool, give_up_callback)
    global velocity 
    velocity = Vector3()
    global bases 
    bases = [["A",0,0,0],["B",0,0,0],["C",0,0,0],["D",-54,-34,-0.5],["E", -19.5,-20,3.5]] 
    bases_visitadas = []
    giveup = False




    #cv_control_publisher = rospy.Publisher("/qrcode_finder/set_running_state", Bool, queue_size=10)
    drone.set_position(-19.5, -21, 8)
    drone.set_position(-19.5, -21, 5)
    for lista in bases:
        if abs(lista[1] + 19.5) < TOL_BASE and abs(lista[2] + 21) < TOL_BASE:
            bases_visitadas.append(str(lista[0]))

    while(qrdetection == False and not giveup):
        for i in range(40):
            #running_pub.publish(Bool(True))
            cv_control_publisher.publish(Bool(True))
            rate.sleep()
    for i in range(60):
        done_pub.publish(Bool(True))
        running_pub.publish(Bool(False))
        cv_control_publisher.publish(Bool(False))
        rate.sleep()
    vel0()
    rospy.loginfo("Bases visitadas: " + str(bases_visitadas))

    while len(bases_visitadas) < 5:
        drone.set_position(drone.controller_data.position.x,drone.controller_data.position.y,30)
        if qrdata in bases_visitadas:
            for lista in bases:
                if lista[0] not in bases_visitadas:
                    x,y,z = coordenadas_base(lista[0])
        else:
            x,y,z = coordenadas_base(qrdata)
        drone.set_position(x,y,30)
        drone.set_position(x,y,z+1)
        
        while(qrdetection == False and not giveup):
            for i in range(40):
                #running_pub.publish(Bool(True))
                cv_control_publisher.publish(Bool(True))
                rate.sleep()
        for i in range(60):
            done_pub.publish(Bool(True))
            running_pub.publish(Bool(False))
            cv_control_publisher.publish(Bool(False))
            rate.sleep()
        for lista in bases:
            if abs(lista[1] - x) < TOL_BASE and abs(lista[2] - y) < TOL_BASE:
                bases_visitadas.append(str(lista[0]))
        vel0()
        rospy.loginfo("Bases visitadas: " + str(bases_visitadas))

def coordenadas_base(letra):
    for lista in bases:
        if lista[0] == letra:
            return lista[1],lista[2],lista[3]


'''def toggle_gripper():
    global drone
    global rate
    global cont
    for i in range(40):
        if cont % 2 == 0:
            drone.gripper("down")
        else:
            drone.gripper("up")
        rate.sleep()
    now = rospy.get_rostime()
    while not rospy.get_rostime() - now > rospy.Duration(secs=6):
        rate.sleep()
    cont +=1'''

def tempo(t):
    now = rospy.get_rostime()
    while not rospy.get_rostime() - now > rospy.Duration(secs=t):
        rate.sleep()

def vel0():
    velocity.x = velocity.y = velocity.z = 0
    for j in range(40):
        vel_publisher.publish(velocity)
        rate.sleep()

if __name__ == "__main__":
    run()

