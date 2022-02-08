#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from MRS_MAV import MRS_MAV
from std_msgs.msg import Bool
from pickle import FALSE
import time
from skyrats_cbr_2021.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Vector3
from mrs_msgs.msg import PositionCommand

MIN_LAR = 1200      #Parametro que define a quantidade minima de pixels laranja para que o drone detecte o tubo
TOL_BASE = 8        

VEL_CERTO_X = 0.2   #Caso o drone receba essa velocidade como input, ele se considera centralizado na cruz
VEL_CERTO_Y = 0.2

class fase1:
    def __init__(self,mav):
        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)  #Recebe informacoes do centro da cruz
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10)    
        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10) #Liga a deteccao da cruz
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)   #Recebe topico de imagem do drone
        self.bridge_object = CvBridge()
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image)
        self.mav = mav
        self.bases_moveis_1 =[]
        self.bases_visitadas = 0        
        self.bases_moveis_0 = []
        self.encontrou_lar = 0
        self.land = 0
        self.soma = 0
        self.rate = rospy.Rate(60)
        self.last_time = time.time()


        # Cam Params
        self.image_pixel_width = 752
        self.image_pixel_height = 480

        # Attributes
        self.giveup = 0
        self.delay = 0
        self.is_lost = True
        self.first_lost = 0
        self.flag = 0
        self.done = 0
        self.first = True
        self.velocity = Vector3()
        self.first_detection = 0
        

        self.setpoint_x = self.image_pixel_height/2  # y size
        self.setpoint_y = self.image_pixel_width/2  # x

    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro da cruz e Proximidade da cruz(Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
        self.first_detection = 1

    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8") 

    def precision_land(self):   #Pousa na cruz utilizando conrole proporcional
            self.delay = time.time() - self.last_time # delay = tempo entre uma deteccao da cruz e outra
            self.is_lost = self.delay > 3             # O drone se considera perdido no pouso caso nao veja a cruz por "3" segundos
            if self.first_detection == 1:

                if not self.is_lost:    
                    if self.detection.area_ratio < 0.30:  # Drone ainda esta longe da cruz, realize o controle proporcional para o pouso
                        if(self.flag == 0):
                            rospy.loginfo("Controle Proporcional")
                            self.flag = 1
                        
                        erro_x = self.detection.center_y - self.setpoint_x #O erro e dado pela diferenca entre o ponto central da camera e a localizacao do centro da cruz
                        erro_y = self.detection.center_x - self.setpoint_y 

                        p = 0.01    #Constante proporcional
                         
                        self.velocity.x= -erro_x * p #O input de velocidade e proporcional ao erro e multiplicado pela constante p
                        self.velocity.y = -erro_y * p
                        if self.velocity.x > 1:     #Limita o modulo da velocidade a 1
                            self.velocity.x =1
                        if self.velocity.x < -1:
                            self.velocity.x =-1
                        if self.velocity.y > 1:
                            self.velocity.y =1
                        if self.velocity.y < -1:
                            self.velocity.y =-1

                        if(abs(self.velocity.x) < VEL_CERTO_X and abs(self.velocity.y) < VEL_CERTO_Y): #Caso o drone esteja centralizado em x e y e dada a ordem de descer
                            self.velocity.z = -1
                        else:
                            self.velocity.z = 0
                       

                        for b in range(10):
                            self.vel_publisher.publish(self.velocity) #Publica a velocidade
                            self.rate.sleep()

                    else:   #Se o drone estiver proximo a cruz, self.land = 1
                        self.velocity.x = self.velocity.y = self.velocity.z = 0 
                        for j in range(20):
                            self.vel_publisher.publish(self.velocity)
                            self.rate.sleep()
                        if (self.flag == 1):
                            rospy.loginfo("Cruz encontrada!")
                            rospy.logwarn("Descendo...")
                            self.flag = 0
                        
                        self.land = 1
                else:
                    pass
                    

            self.rate.sleep()

    def tubo(self):
        if self.encontrou_lar == False: #Verifica se o tubo ja foi encontrado
            self.rows, self.cols, b = self.cv_image.shape
            self.cv_image_cortada = self.cv_image[0: int(self.rows - (self.rows*0.2)) , int(self.cols*0.2) : self.cols] #Corta a imagem pois o drone ve seu trem de pouso laranja na imagem
            self.rows, self.cols, b = self.cv_image_cortada.shape
            self.hsv = cv2.cvtColor(self.cv_image_cortada,cv2.COLOR_BGR2HSV)
            lowerblaranja = np.array([0, 110, 230]) #Cria uma mascara que isola a cor do tubo
            upperblaranja = np.array([26, 160, 255])
            mask_laranja = cv2.inRange(self.hsv, lowerblaranja, upperblaranja)    

            soma_lar = 0
            self.encontrou_lar = False
            for i in range (self.rows): #Varre o frame contando o numero de pixels que estao brancos (os que foram isolados pela mascara)
                for j in range (self.cols):
                    if  mask_laranja[i, j] >= 200:
                        soma_lar += 1

            if (soma_lar > MIN_LAR):    #Verifica se tem pixels suficientes para nao se considerar um engano
                self.soma += 1

            if(self.soma > 2):
                self.encontrou_lar = True
                rospy.loginfo("TUBO DETECTADO")
                
    
    def scan(self, number): #Funcao usada para encontrar as bases moveis 
    #Para comportar todo mapa da competicao e conseguir ver os barcos criamos dois pontos estrategicos para analisar 
        if number == 0:
            cord_x = 50
            cord_y = -5
            self.mav.set_position(50, -5, 50, hdg = 0)
            rospy.loginfo("Iniciando Analise do terreno")
            self.time(8)    
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
        if number == 1:
            cord_x = -45
            cord_y = -5
            self.mav.set_position(-45, -5, 50, hdg = 0)
            rospy.loginfo("Iniciando Analise do terreno")
            self.time(8)
            rows, cols, a = self.cv_image.shape
            cv2.circle(self.cv_image, (int(cols * 62/100),int(rows * 45/100)), 75, (0,255,0), -1)
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)

        lowerbBaseMovel = np.array([0, 0, 150]) #Cria mascara de branco para encontrar os barcos
        upperbBaseMovel = np.array([5, 5, 255])
        mask_BaseMovel = cv2.inRange(hsv, lowerbBaseMovel, upperbBaseMovel)

        kernel = np.ones((5,5), np.uint8)
        mask_BaseMovel = cv2.dilate(mask_BaseMovel ,kernel,iterations = 3) #Dilata pixels para criar contornos mais definidos
        contours, hierarchy = cv2.findContours(mask_BaseMovel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            M = cv2.moments(cnt)
            try: 
                cx = int(M['m10']/M['m00']) #Passa o ponto do frame que foram encontradas os barcos
                cy = int(M['m01']/M['m00'])
            except ZeroDivisionError:
                continue
            cv2.drawContours(self.cv_image, contours, -1, (0, 255, 0), 3)
            #cv2.imshow('cv_image',self.cv_image)
            cv2.waitKey(15)
            
        
            one_pixel_in_meters = 0.28      #Foi visto empiricamente que a uma altura de 50 metros cada pixel da imagem equivale a 0,28 metros
            image_center_x = self.image_pixel_width/2   
            image_center_y = self.image_pixel_height/2

            dif_x = cx - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
            dif_y = cy - image_center_y

            meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
            meters_x = dif_y * one_pixel_in_meters

            cord_base_x = cord_x - meters_x #Coordenada da base movel
            cord_base_y = cord_y - meters_y

            if number == 0:
                self.bases_moveis_0.append([cord_base_x,cord_base_y]) #Por fim, adicionamos a uma lista as coordenadas de cada base
            elif number == 1:
                self.bases_moveis_1.append([cord_base_x,cord_base_y])


            print("Base localizada: " + str(cord_base_x) + " , " + str(cord_base_y))


    def trajectory(self):
        self.scan(0)    #Manda o drone para uma posicao estrategica em busca de bases moveis
        for base in self.bases_moveis_0:    #Executa uma rotina para case base encontrada
            self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 28,1.57) #Sobe o drone para que ele nao bata na base petrolifera
            x,y = base
            rospy.loginfo("Indo para " + str(x) + " , " + str(y))
            self.giveup = False
            self.mav.set_position(x,y,28,1.57) #Manda o drone para a base encontrada 
            self.mav.set_position(x,y,-6,1.57)
            self.landing_control() #Executa a rotina de pouso com controle proporcional

        if self.bases_visitadas < 3: #Verifica se ja foram encontradas todas as bases moveis 
            self.scan(1) #Manda o drone para outra posicao estrategica em busca de bases moveis

            for base in self.bases_moveis_1:
                self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 28,1.57) #Sobe o drone para que ele nao bata na base petrolifera
                skip = 0
                x,y = base

                for lista in self.bases_moveis_0:
                    if abs(lista[0] - x) < TOL_BASE and abs(lista[1] - y) < TOL_BASE: #Verifica se alguma das bases encontradas ja foi visitada no scan(0)
                        rospy.loginfo("Ja foi visitada a base " + str(x) + " , " +str(y))
                        skip = 1

                if skip == 0:   
                    rospy.loginfo("Indo para " + str(x) + " , " + str(y))
                    self.giveup = False
                    self.mav.set_position(x,y,28,1.57)  #Manda o drone para a base encontrada 
                    self.mav.set_position(x,y,-6,1.57)
                    self.landing_control()

        #Apos pousar em todas as bases moveis, inia-se a rotina pre setada de visita as bases fixas 
        self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 28,1.57)  # Sobe o drone para que ele nao bata na base petrolifera
        self.mav.set_position(-53.7, -35.2, 28,1.57)

        rospy.loginfo("Indo para offshore2")
        self.go_to_fix("offshore2")
        self.landing()  
        rospy.loginfo("Procurando Tubo...")
        self.mav.set_position(-50, -36, 4,1.57)
        for i in range(40):
            self.tubo() #Apos chegar na offshore, verifica a prensenca do tubo
            self.rate.sleep()
        self.mav.set_position(-50, -21, 4,1.57)
        rospy.loginfo("Indo para offshore1")
        self.go_to_fix("offshore1")
        self.landing()
        rospy.loginfo("Indo para o pier")
        self.go_to_fix("pier")
        self.landing()

        self.mav.set_position(self.mav.controller_data.position.x, self.mav.controller_data.position.y, 9,1.57)
        self.mav.set_position(10,90,9,1.57) #Volta para a base costeira
        self.mav.altitude_estimator("HEIGHT")   #Troca o sensor de altura do barometro para o lidar (para obter mais precisao)
        self.mav.set_position(10,90,0.55,1.57)
        self.mav.land()    
        self.time(4)
        self.mav.disarm()



    def landing(self):   # Pousa, desarma, arma e decola 
        self.mav.land()    
        self.time(8)
        self.mav.disarm()
        self.bases_visitadas += 1
        rospy.loginfo("N bases visitadas: " + str(self.bases_visitadas))
        self.time(4)
        self.mav.arm()
        self.time(4)
        self.mav.takeoff()
        self.time(6)
        self.mav.altitude_estimator("BARO") # Troca o sensor de altura do lidar para o barometro


    def landing_control(self):  # Pousa utilizando controle proporcional, desarma, arma e decola
        now = rospy.get_rostime()
        self.giveup = 0
        while self.land == 0 and not self.giveup :  
            if (rospy.get_rostime() - now > rospy.Duration(secs=60)): # Caso o drone nao consiga pousar em 60seg --> desiste do pouso
               print("Desisto dessa base")
               self.giveup = 1
            for i in range(40):
                self.cv_control_publisher.publish(Bool(True))   # Ativa a procura pela cruz
                self.rate.sleep()
            self.precision_land()   # Chama a funcao resposavel pelo pouso com controle proporcional
        self.land = 0

        for i in range(40):
            self.cv_control_publisher.publish(Bool(False))  
            self.rate.sleep()
        self.bases_visitadas += 1
        if self.giveup == 0:
            self.mav.land()
            self.time(1)
            self.mav.disarm()
            rospy.loginfo("N bases visitadas: " + str(self.bases_visitadas))
            self.time(6)
            self.mav.arm()
            self.time(4)
            self.mav.takeoff()
            self.time(6)
            self.mav.altitude_estimator("BARO")



    def time(self, t):  #Funcao criada para dar um delay no codigo por N segundos
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=t):
            self.rate.sleep()


    def go_to_fix(self, base):  #Trajetorias predefinidas para as bases fixas
        if base == "pier":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(45.15, 10, 4,1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(45.15, 10, 0.55,1.57)

        if base == "offshore1":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(-19.10, -21.1, 4,1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(-19.10, -21.1, 0.55,1.57)

        if base == "offshore2":
            self.mav.altitude_estimator("BARO")
            self.mav.set_position(-53.7, -35.2, 3,1.57)
            self.mav.altitude_estimator("HEIGHT")
            self.mav.set_position(-53.7, -35.2, 0.55,1.57)


if __name__ == "__main__":
    rospy.init_node("fase1")
    mav = MRS_MAV("uav1")
    missao = fase1(mav)
    missao.trajectory()
