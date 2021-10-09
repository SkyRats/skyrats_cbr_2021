import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge,CvBridgeError
import time
from MRS_MAV import MRS_MAV


class pipeline_scanner:
    def __init__(self, mav):
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.lidar_sub = rospy.Subscriber("/uav1/garmin/range", Range, self.lidar_callback)
        self.bridge_object = CvBridge()
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.mav = mav

    def camera_callback(self, data):
        cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        rows, cols, a = cv_image.shape
        self.cv_image = cv_image[0: int(rows - (rows*0.2)) , int(cols*0.2) : cols]
        self.hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
    
    def lidar_callback(self,data):
        self.lidar_range = data.range

    def detect_red(self):
        lowerbvermelho = np.array([0, 230, 230])
        upperbvermelho = np.array([30, 255, 255])
        self.mask_vermelho = cv2.inRange(self.hsv, lowerbvermelho, upperbvermelho)    
        cv2.imshow('mask_vermelho', self.mask_vermelho)
        cv2.waitKey(3)
        if sum(sum(self.mask_vermelho)) > 0:
            return True

    def detect_green(self):
        lowerbverde = np.array([55, 230, 230])
        upperbverde = np.array([65, 255, 255])
        self.mask_verde = cv2.inRange(self.hsv, lowerbverde, upperbverde)
        cv2.imshow('mask_final_verde', self.mask_verde)
        cv2.waitKey(3)
    
    def sensors_location(self):
        final_mask = self.mask_verde + self.mask_vermelho
        contours, hierarchy = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        locations = []
        for cnt in contours:
            M = cv2.moments(cnt)
            try: 
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                locations.append((cx,cy))
            except ZeroDivisionError:
                pass
        cv2.imshow('soma', final_mask)
        cv2.waitKey(30)
        return locations

    def height_check(self):
        print(self.lidar_range)
        if self.lidar_range < 0.65:
            print("SUBINDOOOOOOOOOOOOOOO")
            self.mav.set_position( 0, 0, 0.4, relative_to_drone=True)

    def run(self):
        rospy.loginfo("Indo para o tubo")
        self.mav.set_position(-49.5, -25, 1.425, relative_to_drone=False)
        time.sleep(10)
        rospy.loginfo("Cheguei, descendo")
        self.mav.set_position(-49.5, -25, -2.3, relative_to_drone=False)
        rospy.loginfo("Em posição, iniciando scan")
        old_locations = []
        locations = []
        while self.mav.controller_data.position.y > -43:
            self.height_check()
            self.mav.set_position(-49.5, self.mav.controller_data.position.y - 0.1, -2.3, relative_to_drone=False)
            if self.detect_red():
                self.mav.set_position(-49.5, self.mav.controller_data.position.y - 0.1, -2.3, relative_to_drone=False)
                t0 = time.time()
                t1 = time.time()
                while t0 - t1 < 3:
                    print('\a')
                    t1 = time.time()
            self.detect_green()
            locations = self.sensors_location()
            aux = []
            for sensor in locations:
                print(old_locations)
                detectado = False
                for old_sensor in old_locations:
                    if int(((sensor[0] - old_sensor[0])**2 + (sensor[1] - old_sensor[1])**2)**(1/2)) < 40:
                        print("esse ja foi detectado!!!")
                        detectado = True
                        break
                if not detectado:
                    print("ESSE É NOVO UHUL")
                aux.append(sensor)
            old_locations = aux
            self.mav.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pipeline_node')
    drone = MRS_MAV("uav1")
    a = pipeline_scanner(drone)
    a.run()