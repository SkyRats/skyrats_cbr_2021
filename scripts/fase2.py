import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge,CvBridgeError
from MRS_MAV import MRS_MAV


class pipeline_scanner:
    def __init__(self, mav):
        self.mav = mav #Objeto mav eh passado para dentro da classe, permitindo controle do drone
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback) #subscriber da camera do drone
        self.lidar_sub = rospy.Subscriber("/uav1/garmin/range", Range, self.lidar_callback) #subscriber do lidar de altura do drone
        self.bridge_object = CvBridge() #Usado para transformar a msg de ROS da camera em uma mensagem legivel para o OpenCV
        rospy.wait_for_message("/uav1/bluefox_optflow/image_raw", Image) #Garante que a imagem eh recebida antes do codigo comecar
        #Inicializacao de variaveis
        self.sensors_locations = {}
        self.old_values = [(0,0),(0,0),(0,0),(0,0),(0,0)]
        self.sensors_colors = {}

    def camera_callback(self, data):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8") #self.cv_image recebe imagens da camera constantemente
        self.mav.rate.sleep()

    def lidar_callback(self,data):
        self.lidar_range = data.range #self.lidar_range recebe a altura do drone constantemente

    def detect_red(self):
        #alcance das mascaras de cor
        lowerbvermelho = np.array([0, 50, 20])
        upperbvermelho = np.array([5, 255, 255])
        self.mask_vermelho = cv2.inRange(self.hsv, lowerbvermelho, upperbvermelho)
        cv2.waitKey(15)

        if sum(sum(self.mask_vermelho)) > 1000: #entra se houver vermelho o suficiente na imagem
            contours, hierarchy = cv2.findContours(self.mask_vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #pega os contornos das formas vermelhas
            for cnt in contours:
                M = cv2.moments(cnt) #pega o centro dos contornos vermelhos
                try: 
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                except ZeroDivisionError:
                    continue
                aux = self.which_sensor_is_this((cx,cy)) #testa para ver se esse sensor ja foi avistado antes
                if aux == None: #caso seja um sensor novo
                    aux = len(self.sensors_locations)
                    self.sensors_locations[len(self.sensors_locations)] = (cx,cy)
                    self.sensors_colors[len(self.sensors_locations)] = "vermelho"
                    rospy.loginfo("Sensor " + str(aux) +": Vermelho!!!") #aviso no terminal
                else: #caso seja um sensor ja conhecido
                    self.sensors_locations[aux] = (cx,cy) #atualiza a localizacao dele
                cv2.putText(self.debug_image, "Sensor " + str(aux), (cnt[0][0][0] - 20,cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            cv2.drawContours(self.debug_image, contours, -1, (255,0,0), 2) #debug requisitado na competicao
        return


    def detect_green(self):
        #alcance das mascaras de cor
        lowerbverde = np.array([55, 230, 230])
        upperbverde = np.array([65, 255, 255])
        self.mask_verde = cv2.inRange(self.hsv, lowerbverde, upperbverde)

        if sum(sum(self.mask_verde)) > 1000: #entra se houver verde o suficiente na imagem
            contours, hierarchy = cv2.findContours(self.mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #pega os contornos das formas verdes
            for cnt in contours:
                M = cv2.moments(cnt) #pega o centro dos contornos verdes
                try: 
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                except ZeroDivisionError:
                    continue
                aux = self.which_sensor_is_this((cx,cy)) #checa se o sensor eh conhecido
                if aux == None: #caso seja novo
                    aux = len(self.sensors_locations) 
                    self.sensors_locations[len(self.sensors_locations)] = (cx,cy)
                    self.sensors_colors[len(self.sensors_locations)] = "verde"
                    rospy.loginfo("Sensor " + str(aux) +": Verde!!!")
                else: #caso seja conhecido
                    self.sensors_locations[aux] = (cx,cy) #atualiza a localizacao do sensor
                cv2.putText(self.debug_image, "Sensor " + str(aux), (cnt[0][0][0] - 20,cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            cv2.drawContours(self.debug_image, contours, -1, (255,0,0), 2) #debug requisitado pela competicao
        return

    def which_sensor_is_this(self, tuple): #funcao que verifica se um sensor ja eh conhecido
        locations = self.sensors_locations.items()
        for sensor_location in locations:
            if ((sensor_location[1][0] - tuple[0])**2 + (sensor_location[1][1] - tuple[1])**2)**(1/2) < 30:
                return sensor_location[0]
        return None

    def height_check(self): #devolve a altura atual do drone como aparecer no lidar de altura
        print("Altura atual do drone: " + str(self.lidar_range))
    
    def sensor_reset(self): #retira da lista de conhecidos os sensores que ja estao fora da tela (ja passaram)
        for i in range(len(self.sensors_locations)):
            if self.sensors_locations[i] == self.old_values[i]:
                self.sensors_locations[i] = (-99,-99)
            else:
                self.old_values[i] = self.sensors_locations[i]

    def run(self): #arquitetura da missao, explicado atraves do terminal
        rospy.loginfo("Indo para o tubo")
        self.mav.set_position(-49.6, -24.7, 3, relative_to_drone=False)
        rospy.loginfo("Cheguei, descendo")
        self.mav.altitude_estimator("HEIGHT")
        self.mav.set_position(-49.6, -24.7, 1, relative_to_drone=False)
        rospy.loginfo("Em posicao, iniciando scan")
        while self.mav.controller_data.position.y > -44.5:
            self.debug_image = self.cv_image
            self.hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
            self.height_check()
            self.mav.set_position(-49.6, self.mav.controller_data.position.y - 0.15, 1, relative_to_drone=False)
            self.detect_green()
            self.detect_red()
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
        self.mav.disarm() #disarma quando a altura do chao for menor do que 0.25m

if __name__ == '__main__':
    rospy.init_node('pipeline_node')
    drone = MRS_MAV("uav1")
    a = pipeline_scanner(drone)
    a.run()