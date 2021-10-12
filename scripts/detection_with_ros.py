#!/usr/bin/env python3

import rospy
import mavros_msgs
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from pyzbar.pyzbar import decode
import numpy as np

class QRDetection():

    def __init__(self):
        self.rate = rospy.Rate(60)
        self.bridge_object = CvBridge()

        self.detection = False
        self.running_state = False

        self.cam_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw/", Image, self.cam_callback)
        self.running_sub = rospy.Subscriber("/qrdetection/set_running_state", Bool, self.running_callback)

        self.detection_pub = rospy.Publisher("/qrdetection/detection_status", Bool, queue_size=10)
        self.data_pub = rospy.Publisher("/qrdetection/qr_data", String, queue_size=10)

        # SUBSCRIBERS
        self.detection = 0
        self.running_state = False
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)

    def cam_callback(self, data):
        try:
            self.cam_frame = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)


    def running_callback(self, bool):
        self.running_state = bool.data

    def detect(self):
        barcode_antigo = None
        rospy.loginfo("Starting QR Detection")
        while not rospy.is_shutdown():
            if self.running_state:
                self.detection = 0
                gray = cv2.cvtColor(self.cam_frame, cv2.COLOR_BGR2GRAY)
                img_bw = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)[1] #thresh = 40

                qr_result = decode(img_bw)
                for barcode in qr_result:
                    for i in range(20):
                        self.detection_pub.publish(Bool(True))
                        self.rate.sleep()
                    self.detection += 1
                    (x, y, w, h) = barcode.rect
                    cv2.rectangle(self.cam_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                    # the barcode data is a bytes object so if we want to draw it
                    # on our output image we need to convert it to a string first
                    barcodeData = barcode.data.decode("utf-8")
                    if(barcodeData != barcode_antigo):
                        barcode_antigo = barcodeData
                        flag = 0
                    for i in range(20):
                        self.data_pub.publish(String(barcodeData))
                        self.rate.sleep()
                    if(flag == 0):
                        #rospy.loginfo("Type: " + str(barcodeType))
                        rospy.loginfo("QRCODE: " + str(barcodeData))
                        flag = 1
                    self.rate.sleep()
            else:
                for i in range(10):
                    self.detection_pub.publish(Bool(False))
                self.rate.sleep()




if __name__ == "__main__":
    rospy.init_node("qr_detection")
    detection = QRDetection()
    detection.detect()