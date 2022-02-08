#!/usr/bin/env python3


from pickle import FALSE, TRUE
import rospy
import mrs_msgs
from mrs_msgs import srv
from mrs_msgs.msg import PositionCommand, Reference, TrajectoryReference
from mrs_msgs.srv import TrajectoryReferenceSrv, ReferenceStampedSrv, String
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from std_srvs import srv
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from mavros_msgs.srv import CommandBool,CommandHome
from mavros_msgs.srv import SetMode
from controller_g_msgs.srv import GetStance
from std_msgs.msg import Bool


import numpy as np
import math
import time
import os
GRIPPER = True
TOL = 0.1

class MRS_MAV:
    def __init__(self, mav_name):
        self.rate = rospy.Rate(60)
        self.hz = 60
        self.controller_data = PositionCommand()
        self.position_controller = Reference()
        self.position_controller_header = Header()
        self.trajectory = TrajectoryReference()
        self.mav_name = mav_name
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.cont = 0
        self.flag = 0

        ############# Services #############
        self.reference = rospy.ServiceProxy("/" + mav_name + "/control_manager/reference", ReferenceStampedSrv)
        self.trajectory_reference_srv = rospy.ServiceProxy("/" + mav_name + "/control_manager/trajectory_reference", TrajectoryReferenceSrv)
        self.goto_trajectory_start = rospy.ServiceProxy("/" + mav_name + "/control_manager/goto_trajectory_start", Trigger)
        self.start_trajectory_tracking = rospy.ServiceProxy("/" + mav_name + "/control_manager/start_trajectory_tracking", Trigger)
        self.stop_trajectory_tracking = rospy.ServiceProxy("/" + mav_name + "/control_manager/stop_trajectory_tracking", Trigger)
        self.resume_trajectory_tracking = rospy.ServiceProxy("/" + mav_name + "/control_manager/resume_trajectory_tracking", Trigger)
        self.takeoff_srv = rospy.ServiceProxy("/" + mav_name +"/uav_manager/takeoff", Trigger)
        self.land_srv = rospy.ServiceProxy("/" + mav_name +"/uav_manager/land", Trigger)
        self.disarm_srv = rospy.ServiceProxy("/" + mav_name + "/control_manager/arm", SetBool) #Esse com argumento 0 disarma o drone
        self.arm_srv = rospy.ServiceProxy("/" + mav_name + "/mavros/cmd/arming", CommandBool )
        self.motors_srv = rospy.ServiceProxy("/" + mav_name + "/control_manager/motors", SetBool)
        self.set_mode_srv = rospy.ServiceProxy("/" + mav_name + "/mavros/set_mode", SetMode)
        self.gripper_srv = rospy.ServiceProxy("/" + mav_name + "/control_manager/controller_gripper", GetStance)
        self.set_home_srv = rospy.ServiceProxy("/" + mav_name + "/mavros/cmd/set_home", CommandHome)
        self.change_alt_estimator = rospy.ServiceProxy("/" + mav_name + "/odometry/change_alt_estimator_type_string", String)
        self.garmin_toggle = rospy.ServiceProxy("/" + mav_name + "/odometry/toggle_garmin", SetBool)
        rospy.wait_for_service("/" + mav_name + "/odometry/toggle_garmin")
        #self.garmin_toggle(False)


        ############# Subscribers #############
        self.cv_control_sub = rospy.Subscriber("/precision_landing/set_running_state", Bool, self.cv_callback)
        self.stop_sub = rospy.Subscriber("/stop_trajectory", Bool, self.stop_callback)

        controller_sub = rospy.Subscriber("/" + mav_name + "/control_manager/position_cmd", PositionCommand, self.controller_callback)
        vel_sub = rospy.Subscriber("/vel", Vector3, self.vel_callback)
        #rospy.wait_for_message("/" + mav_name + "/control_manager/position_cmd", PositionCommand)
        self.init_x = self.controller_data.position.x
        self.init_y = self.controller_data.position.y
        '''print("x")
        print(self.controller_data.position.x)
        print("y")
        print(self.controller_data.position.y)'''

    ############# Callback Functions #############

    def cv_callback(self, data):
        self.run = data.data


    def stop_callback(self, data):      #Recebe ordem para parar a trajetoria
        self.stop = data.data
        if(self.stop == 1):
            self.flag = 1
            


    def controller_callback(self, data):
        self.controller_data = data
    
    def vel_callback(self, data):
        self.vel_x = data.x
        self.vel_y = data.y
        self.vel_z = data.z

    ############# Drone Functions #############
    def set_position(self, x, y, z=None, hdg=None, relative_to_drone = False):
        if z == None:
            z = self.controller_data.position.z
        if hdg == None:
            hdg = self.controller_data.heading
        
        if relative_to_drone:
            self.position_controller_header.frame_id = "fcu_untilted"
            self.position_controller_header.stamp = rospy.Time(0)
        
     
        self.position_controller.position.x = x
        self.position_controller.position.y = y
        self.position_controller.position.z = z
        self.position_controller.heading = hdg
        if(self.vel_x == 0 and self.vel_y == 0 and self.vel_z == 0 and relative_to_drone == False):
            rospy.wait_for_service("/" + self.mav_name + "/control_manager/reference")

            while (abs(self.controller_data.position.x - x) > TOL or abs(self.controller_data.position.y - y) > TOL or abs(self.controller_data.position.z - z) > TOL) and self.flag == 0:
                self.reference(self.position_controller_header, self.position_controller)

        else:
            self.reference(self.position_controller_header, self.position_controller)
        self.flag = 0
   
   
    def input_trajectory(self, trajectory,dt, use_heading = False, loop = False ,relative_to_drone = False):
        self.trajectory.header.stamp = rospy.Time(0)
        if relative_to_drone:
            self.trajectory.header.frame_id = "fcu_untilted"
        
        self.trajectory.use_heading = use_heading #whether the heading should be tracked or not
        self.trajectory.fly_now = False
        self.trajectory.loop = loop #whether the trajectory should be looped or not
        self.trajectory.dt = dt #time between points, if it can't get there it will just aim at the next
        #trajectory input in the format [[x1,y1,z1,hdg1],[x2,y2,z2,hdg2]...] or [[x1,y1,z1],[x2,y2,z2]...]
        ReferenceList = []
        for point in trajectory:
            current_point = Reference()
            current_point.position.x = point[0]
            current_point.position.y = point[1]
            current_point.position.z = point[2]
            if use_heading:
                current_point.heading = point[3]
            ReferenceList.append(current_point)
        self.trajectory.points = ReferenceList


        rospy.wait_for_service("/" + self.mav_name + "/control_manager/trajectory_reference")
        self.trajectory_reference_srv(self.trajectory)
        rospy.wait_for_service("/" + self.mav_name + "/control_manager/start_trajectory_tracking")
        self.start_trajectory_tracking()

    
    
    def land(self):
        rospy.wait_for_service("/" + self.mav_name +"/uav_manager/land")
        land_output = self.land_srv()
        if land_output.success:   
            rospy.loginfo_once(land_output.message)
        else:
            rospy.logerr_once(land_output.message)
 
    def RTL(self):
        self.position_controller.position.x = 10
        self.position_controller.position.y = 90
        self.position_controller.position.z = 6
        self.position_controller.heading = 0
        
        rospy.wait_for_service("/" + self.mav_name + "/control_manager/reference")
        while abs(self.controller_data.position.x - 10) > TOL or abs(self.controller_data.position.y - 90) > TOL or abs(self.controller_data.position.z - 6) > TOL:
            self.reference(self.position_controller_header, self.position_controller)
        
    def takeoff(self):
        rospy.wait_for_service("/" + self.mav_name +"/uav_manager/takeoff")
        takeoff_output = self.takeoff_srv()
        if takeoff_output.success:
            rospy.loginfo_once(takeoff_output.message)
        else:
            rospy.logerr_once(takeoff_output.message)

    def disarm(self):
        rospy.wait_for_service("/" + self.mav_name +"/control_manager/arm")
        disarm_output = self.disarm_srv(0)
        if disarm_output.success:
            rospy.loginfo_once(disarm_output.message)
        else:
            rospy.logerr_once(disarm_output.message)

    def arm(self):
        rospy.wait_for_service("/" + self.mav_name +"/control_manager/motors")
        motors_output = self.motors_srv(1)
        if motors_output.success:
            rospy.loginfo_once(motors_output.message)
        else:
            rospy.logerr_once(motors_output.message)
        
        rospy.wait_for_service("/" + self.mav_name +"/mavros/cmd/arming")
        arm_output = self.arm_srv(1)
        if arm_output.success:
            rospy.loginfo_once(arm_output.success)
        else:
            rospy.logerr_once(arm_output.success)

        set_mode_output = self.set_mode_srv(custom_mode='OFFBOARD')
        if set_mode_output:
            rospy.loginfo_once(set_mode_output)
        else:
            rospy.logerr_once(set_mode_output)

    '''def set_home(self):
        yaw = 0
        latitude = 0#47.3985545
        longitude =0#8.5457273
        altitude = 0#64.03211528472812


        rospy.wait_for_service("/" + self.mav_name + "/mavros/cmd/set_home")
        self.set_home_srv(yaw, latitude, longitude, altitude)'''

    def gripper(self, op): # op: up, down, close, open
        if(GRIPPER == 1):
            rospy.wait_for_service("/" + self.mav_name + "/control_manager/controller_gripper")
            self.gripper_srv(op)
            rospy.loginfo_once(op + " Done")
            
        else:
            print("Garra desligada")
    
    def altitude_estimator(self, value): #"BARO" or "HEIGHT"
        #if value == "HEIGHT":
        #    rospy.wait_for_service("/" + self.mav_name + "/odometry/toggle_garmin")
        #    print(self.garmin_toggle(True))
        #else:
        #    rospy.wait_for_service("/" + self.mav_name + "/odometry/toggle_garmin")
        #    print(self.garmin_toggle(False))
        rospy.wait_for_service("/" + self.mav_name + "/odometry/change_alt_estimator_type_string")
        self.change_alt_estimator(value)
        
    
    def run(self):
        s = 0.5
        while not rospy.is_shutdown():
            if((self.vel_x != 0 or self.vel_y != 0 or self.vel_z != 0)):
                now = rospy.get_rostime()
                while not rospy.get_rostime() - now > rospy.Duration(secs=s):
                    self.rate.sleep()
                self.set_position((self.vel_x * s),(self.vel_y * s),(self.vel_z * s ),0, True)
            self.rate.sleep()
        
        
if __name__ == '__main__':
    rospy.init_node("MRS_MAV")
    mav = MRS_MAV("uav1")
    #mav.land()
    #mav.disarm()
    #mav.arm()
    #mav.takeoff()
    #mav.set_position(0, 0, 25, 0, relative_to_drone=True) # 55, -18, 50    -50, -18, 50
    #mav.gripper("down")
    #mav.set_position(30, -17, 0,0) # 55, -18, 50    -50, -18, 50
    #mav.altitude_estimator("HEIGHT")
    mav.run()
    #mav.RTL()
