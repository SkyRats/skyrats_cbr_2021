#!/usr/bin/env python3
import cv2
import time
import rospy
import mavros_msgs
import numpy as np
from pickle import FALSE
from MRS_MAV import MRS_MAV
from traceback import print_tb
from pyzbar.pyzbar import decode
from rospy.topics import Publisher
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
import qr_detection as Detection
from skyrats_cbr_2021.msg import H_info
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image, Range


TOL=0.2
TOL_BASE = 5
VEL_CERTO_X = 0.25
VEL_CERTO_Y = 0.25

#Esse codigo visita as bases encontradas na fase1 e analisa qrcodes em cada uma delas
class fase4: 

    def __init__(self,mav):
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)    
        self.drone = mav
        self.rate = rospy.Rate(60)
        self.last_time = time.time()
        self.bridge_object = CvBridge()

        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10) #Liga a procura pela cruz
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10) 
        
        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback) # Recebe dados do centro da cruz na imagem
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback) # Recebe o topico de imagem da camera


        # Attributes
        self.velocity = Vector3()
        self.giveup = False
        self.is_lost = True
        self.qrdata = None
        self.first_lost = 0
        self.land = 0
        self.decodificou = 0
        self.delay = 0
        self.flag = 0
        self.qr_encontrado = 0  
        self.perto_qr = 0
        self.barcode_antigo = None

        self.repetido = 0
        self.setpoint_x = 480/2  # y size
        self.setpoint_y = 752/2  # x



    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        if self.perto_qr == 1:
            self.decode_qr()

    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()


    def decode_qr(self):    #Decodifica o qrcode
        self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #img_bw = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)[1] #thresh = 40


        qr_result = decode(self.gray)
        for barcode in qr_result:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # the barcode data is a bytes object so if we want to draw it
            # on our output image we need to convert it to a string first
            self.barcodeData = barcode.data.decode("utf-8")
            self.decodificou = 1            

            if(self.barcodeData != self.barcode_antigo):    #Evita que a mensagem de decodificacao seja spamada
                self.barcode_antigo = self.barcodeData
                self.repetido = 0

            if(self.repetido == 0):
                rospy.loginfo("QRCODE: " + str(self.barcodeData))
                self.repetido = 1


    def boat_centralize(self):  #Realiza controle proporcional para manter o drone centralizado na base para que ele consiga identificar o qrcode
        if self.decodificou == 0:   #Verifica se a deteccao ja foi feita 
            self.perto_qr = 1 
            self.delay = time.time() - self.last_time #Tempo entre uma deteccao da cruz e outra
            self.is_lost = self.delay > 3
            if not self.is_lost:
                
                if(self.flag == 0):
                    rospy.loginfo("Controle PID")
                    self.flag = 1
                
                erro_x = self.detection.center_y - self.setpoint_x #Erro --> diferenca entre centro da imagem e posicao da cruz na imagem
                erro_y = self.detection.center_x - self.setpoint_y 

                p = 0.012
                    
                self.velocity.x= -erro_x * p
                self.velocity.y = -erro_y * p
                if self.velocity.x > 1: #Limita a velocidade a 1
                    self.velocity.x =1
                if self.velocity.x < -1:
                    self.velocity.x =-1
                if self.velocity.y > 1:
                    self.velocity.y =1
                if self.velocity.y < -1:
                    self.velocity.y =-1
                if self.detection.area_ratio < 0.45:  # Drone ainda esta longe da cruz

                    if(abs(self.velocity.x) < VEL_CERTO_X and abs(self.velocity.y) < VEL_CERTO_Y): #Caso o resultado do controle seja suficientemente baixo, considera-se o drone centralizado
                        self.velocity.z = -1 
                    else:
                        self.velocity.z = 0
                
                else:
                    self.velocity.z = 0

                for b in range(10):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()

               
            else:
                rospy.loginfo("Base fora do campo de visao")
            self.rate.sleep()


    def run(self):
   
        rospy.loginfo("Indo para base movel 1!")
        self.go_to_fix("movel1")
        self.boat_detect()
        self.vel0()
        self.drone.set_position(-25, 30, 4, hdg=1.57)
        rospy.loginfo("Indo para base movel 2!")
        self.vel0()
        self.go_to_fix("movel2")
        self.boat_detect()
        #self.drone.set_position(60, 0, 4, hdg=1.57)
        self.drone.set_position(69, 0, 4, hdg=1.57)

        rospy.loginfo("Indo para base movel 3!")
        self.vel0()
        self.go_to_fix("movel3")
        self.boat_detect()
        rospy.loginfo("Set position")
        self.vel0()
        #self.drone.set_position(30, -55, 7, hdg=1.57)
        self.drone.set_position(33, -55, 4, hdg=1.57)
        rospy.loginfo("Indo para base offshore1")
        self.go_to_fix("offshore1")
        self.fix_detect()
        rospy.loginfo("Indo para base offshore2")
        self.go_to_fix("offshore2")
        self.fix_detect()
        rospy.loginfo("Voltando para a costeira")
        self.drone.set_position(self.drone.controller_data.position.x, self.drone.controller_data.position.y, 9,1.57)
        self.drone.set_position(10,90,9,1.57)   #Volta para a base costeira
        self.drone.altitude_estimator("HEIGHT")
        self.drone.set_position(10,90,0.55,1.57)
        self.drone.land()    
        self.tempo(4)
        self.drone.disarm()




    def tempo(self,t):  #Espera N segundos sem fazer nada
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.rate.sleep()


    def fix_detect(self):   #Usada para detectar o qrcode em bases fixas, nao utiliza controle por ja saber a posicao exata da base
        self.decodificou = 0
        self.giveup = 0
        now = rospy.get_rostime()
        while self.decodificou == 0 and not self.giveup :
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)):   #Caso nao ache o qrcode em 60seg --> desiste
               print("Desisto dessa base")
               self.giveup = 1
            self.decode_qr()    
        self.decodificou = 0
        self.drone.altitude_estimator("BARO")


    def boat_detect(self):  #Usada para detectar o qrcode em bases moveis,  utiliza controle por conta do movimento do barco
        now = rospy.get_rostime()
        self.giveup = 0
        self.decodificou = 0
        while self.decodificou == 0 and not self.giveup :
            if (rospy.get_rostime() - now > rospy.Duration(secs=90)):#Caso nao ache o qrcode em 90seg --> desiste
               print("Desisto dessa base")
               self.giveup = 1
            for i in range(40):
                self.cv_control_publisher.publish(Bool(True))
                self.rate.sleep()
            self.boat_centralize()
        for i in range(40):
            self.cv_control_publisher.publish(Bool(False))
            self.rate.sleep()
     
        self.perto_qr = 0
        self.decodificou = 0


    def go_to_fix(self, base):
        if base == "offshore1":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(-19.10, -21.1, 7, hdg= 1.57)
            self.drone.altitude_estimator("HEIGHT")
            self.drone.set_position(-19.10, -21.1, 0.55)

        if base == "offshore2":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(-50, -21, 4, hdg=1.57)
            self.drone.set_position(-53.7, -35.2, 4, hdg=1.57)
            self.drone.altitude_estimator("HEIGHT")
            self.drone.set_position(-53.7, -35.2, 0.55)

        if base == "movel3":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(33, -55, 4, hdg=1.57)
            self.drone.set_position(33, -55, -6, hdg=1.57)
        
        if base == "movel2":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(69, 0, 4, hdg=1.57)
            self.drone.set_position(69, 0, -6, hdg=1.57)
        
        if base == "movel1":
            self.drone.altitude_estimator("BARO")
            self.drone.set_position(-25, 30, 4, hdg=1.57)
            self.drone.set_position(-25, 30, -6, hdg=1.57)


    def vel0(self):
        self.velocity.x = self.velocity.y = self.velocity.z = 0
        for j in range(20):
            self.vel_publisher.publish(self.velocity)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fase4")
    drone = MRS_MAV("uav1")
    a = fase4(drone)
    a.run()

