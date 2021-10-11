#!/usr/bin/env python3

import rospy
import mavros_msgs
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
import numpy as np

class QRDetection():

    def __init__(self):
        self.rate = rospy.Rate(60)
        self.bridge_object = CvBridge()
        self.cam_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw/", Image, self.cam_callback)
        self.detection = 0
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)


    def cam_callback(self, data):
        try:
            self.cam_frame = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

    def detect(self):
        rospy.loginfo("Starting QR Detection")
        self.detection = 0

        (rows,cols,channels) = self.cam_frame.shape
    
        #resized_image = cv2.resize(self.cam_frame, (360, 640)) 

        gray = cv2.cvtColor(self.cam_frame, cv2.COLOR_BGR2GRAY)
        thresh = 40
        img_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

        qr_result = decode(img_bw)
        cv2.imwrite("PrimeiraImagemTratada.jpeg", self.cam_frame)

        while(self.detection < 120):
            for barcode in qr_result:
                self.detection += 1
                (x, y, w, h) = barcode.rect
                cv2.rectangle(self.cam_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                # the barcode data is a bytes object so if we want to draw it
                # on our output image we need to convert it to a string first
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type

                rospy.loginfo("Type: " + str(barcodeType))
                rospy.loginfo("Data: " + str(barcodeData))
                self.rate.sleep()
            self.rate.sleep()



if __name__ == "__main__":

    rospy.init_node("qr_detection")
    detection = QRDetection()
    detection.detect()