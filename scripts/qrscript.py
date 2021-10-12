#!/usr/bin/env python3

import rospy
import mavros_msgs
from std_msgs.msg import Bool, String
import detection_with_ros as Detection
from MRS_MAV import MRS_MAV
import numpy as np
qrdetection = False
qrdata = None
TOL=0.2
global cont 
cont = 0

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
    done_pub = rospy.Subscriber('/precision_landing/done', Bool, queue_size=10)

    #cv_control_publisher = rospy.Publisher("/qrcode_finder/set_running_state", Bool, queue_size=10)
    toggle_gripper()    
    drone.set_position(-19.5, -21, 8)
    toggle_gripper()
    drone.set_position(-19.5, -21, 4)
    while(qrdetection == False):
        for i in range(40):
            running_pub.publish(Bool(True))
            cv_control_publisher.publish(Bool(True))
            rate.sleep()
    for i in range(40):
            running_pub.publish(Bool(False))
            cv_control_publisher.publish(Bool(False))
            rate.sleep()
    drone.set_position(drone.controller_data.position.x,drone.controller_data.position.y,30)

    x,y,z = coordenadas_base(qrdata)
    drone.set_position(x,y,30)
    toggle_gripper()
    drone.set_position(x,y,z+2.5)
    toggle_gripper()


def coordenadas_base(letra):
    bases = [["A",0,0,0],["B",0,0,0],["C",0,0,0],["D",-54,-34,0],["E", -19.5,-20,3.5]] 
    for lista in bases:
        if lista[0] == letra:
            return lista[1],lista[2],lista[3]


def toggle_gripper():
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
    while not rospy.get_rostime() - now > rospy.Duration(secs=5):
        rate.sleep()
    cont +=1

if __name__ == "__main__":
    run()

