import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge,CvBridgeError
import time
from MRS_MAV import MRS_MAV


class pipeline_scanner:
    def __init__(self, mav):
        self.mav = mav
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.lidar_sub = rospy.Subscriber("/uav1/garmin/range", Range, self.lidar_callback)
        self.bridge_object = CvBridge()
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.sensors_locations = {}
        self.sensors_colors = {}
        self.old_values = [(0,0),(0,0),(0,0),(0,0),(0,0)]

    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.debug_image = self.cv_image
        self.rows, self.cols, a = self.debug_image.shape
        #self.cv_image = self.cv_image[int(self.rows*0.3) : int(self.rows - (self.rows*0.3)) , int(self.cols*0.3) : int(self.cols - (self.cols*0.3))]
        self.hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV) #[int(rows*0.4) : int(rows - (rows*0.4)) , int(cols*0.4) : int(cols - (cols*0.4))]
        self.mav.rate.sleep()

    def lidar_callback(self,data):
        self.lidar_range = data.range

    def detect_red(self):
        lowerbvermelho = np.array([0, 230, 230])
        upperbvermelho = np.array([30, 255, 255])
        self.mask_vermelho = cv2.inRange(self.hsv, lowerbvermelho, upperbvermelho)
        if sum(sum(self.mask_vermelho)) > 1000:
            contours, hierarchy = cv2.findContours(self.mask_vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                M = cv2.moments(cnt)
                try: 
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                except ZeroDivisionError:
                    continue
                aux = self.which_sensor_is_this((cx,cy))
                if aux == None:
                    aux = len(self.sensors_locations)
                    self.sensors_locations[len(self.sensors_locations)] = (cx,cy)
                    self.sensors_colors[len(self.sensors_locations)] = "vermelho"
                else:
                    self.sensors_locations[aux] = (cx,cy)
                cv2.putText(self.debug_image, "Sensor " + str(aux), (cnt[0][0][0] - 100,cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)
            cv2.drawContours(self.debug_image, contours, -1, (255,0,0), 2)
        return


    def detect_green(self):
        lowerbverde = np.array([55, 230, 230])
        upperbverde = np.array([65, 255, 255])
        self.mask_verde = cv2.inRange(self.hsv, lowerbverde, upperbverde)
        if sum(sum(self.mask_verde)) > 1000:
            contours, hierarchy = cv2.findContours(self.mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                M = cv2.moments(cnt)
                try: 
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                except ZeroDivisionError:
                    continue
                aux = self.which_sensor_is_this((cx,cy))
                if aux == None:
                    aux = len(self.sensors_locations)
                    self.sensors_locations[len(self.sensors_locations)] = (cx,cy)
                    self.sensors_colors[len(self.sensors_locations)] = "verde"
                else:
                    self.sensors_locations[aux] = (cx,cy)
                cv2.putText(self.debug_image, "Sensor " + str(aux), (cnt[0][0][0] - 100,cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)
            cv2.drawContours(self.debug_image, contours, -1, (255,0,0), 2)
        return

    def which_sensor_is_this(self, tuple):
        locations = self.sensors_locations.items()
        for sensor_location in locations:
            if ((sensor_location[1][0] - tuple[0])**2 + (sensor_location[1][1] - tuple[1])**2)**(1/2) < 30:
                return sensor_location[0]
        return None

    def height_check(self):
        print("Altura atual do drone: " + str(self.lidar_range))
    
    def sensor_reset(self):
        for i in range(len(self.sensors_locations)):
            if self.sensors_locations[i] == self.old_values[i]:
                self.sensors_locations[i] = (-99,-99)
            else:
                self.old_values[i] = self.sensors_locations[i]

    def run(self):
        rospy.loginfo("Indo para o tubo")
        self.mav.altitude_estimator("BARO")
        self.mav.set_position(-49.6, -24.7, 3, relative_to_drone=False)
        rospy.loginfo("Cheguei, descendo")
        self.mav.altitude_estimator("HEIGHT")
        self.mav.set_position(-49.6, -24.7, 1, relative_to_drone=False)
        rospy.loginfo("Em posição, iniciando scan")
        while self.mav.controller_data.position.y > -44.5:
            self.height_check()
            self.mav.set_position(-49.6, self.mav.controller_data.position.y - 0.15, 1, relative_to_drone=False)
            self.detect_red()
            self.detect_green()
            cv2.imshow("camera_drone", self.debug_image)
            self.sensor_reset()
            cv2.waitKey(15)
        rospy.loginfo("Scan finalizado, subindo")
        self.mav.set_position(-49.5, -44, 4, relative_to_drone=False)
        rospy.loginfo("Voltando para a base costeira")
        self.mav.altitude_estimator("BARO")
        self.mav.set_position(-49.5, -25, 4, relative_to_drone=False)
        self.mav.set_position(10, 90, 4, relative_to_drone=False)
        self.mav.altitude_estimator("HEIGHT")
        self.mav.set_position(10, 90, 1.5, relative_to_drone=False)
        rospy.loginfo("Pousando")
        self.mav.land()
        while self.lidar_range > 0.25:
            pass
        self.mav.disarm()

if __name__ == '__main__':
    rospy.init_node('pipeline_node')
    drone = MRS_MAV("uav1")
    a = pipeline_scanner(drone)
    a.run()