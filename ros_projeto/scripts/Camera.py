#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
import sys
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, String
from tf import transformations
from tf import TransformerROS
import tf2_ros
import math
from sensor_msgs.msg import CompressedImage # Subscriber
from sensor_msgs.msg import Image # Publisher
from geometry_msgs.msg import Pose
import json

import visao_module
import regressaoLinear as rl
import corModule as cM

#---------------------------Parâmetros do aruco--------------------------------
#Define the aruco dictionar
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000

#Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN
scan_dist = 0

bridge = CvBridge()

#Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
#Descarta imagens que chegam atrasadas demais

check_delay = False 

frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()

#----------------------------Variáveis do Aruco----------------------------

distancenp = 0

class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(MyEncoder, self).default(obj)


class Camera:
    def __init__(self):
        self.pubCamera = rospy.Publisher("Camera", String, queue_size=1)

        rospy.init_node("Camera", anonymous=True)

        self.bgr = None
        self.media = [0,0]
        self.centro = [0,0]
        self.resultados = None
        self.coef_angular = 0
        self.x_linha = 0 
        self.ids = []
        self.distancenp = 0
        self.contadorFrame = 0
        self.rvec = None

        self.dicCamera = {}

        self.ESTADO = ""
        self.COR = ""
        self.id = 0
        self.CREEPERNAMAO = False
        self.PEGACREEPER = False
        self.OBJETIVONATELA = False

        topico_imagem = "/camera/image/compressed"
        self.image_sub = rospy.Subscriber(topico_imagem,CompressedImage,self.roda_todo_frame)

        self.estado_sub = rospy.Subscriber("Estados", String, self.callback)

        self.marker_size = 20

        self.calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
        self.camera_matrix   = np.loadtxt(self.calib_path+'cameraMatrix_raspi.txt', delimiter=',')
        self.camera_distortion   = np.loadtxt(self.calib_path+'cameraDistortion_raspi.txt', delimiter=',')

        

    def callback(self, estadoDic):
        # converteer para dic!!
        self.estadoDic = json.loads(estadoDic.data)

        self.ESTADO = self.estadoDic['ESTADO']
        self.ESTACAO = self.estadoDic['ESTACAO']
        self.ANDAR = self.estadoDic['ANDAR']
        self.CREEPERNAMAO = self.estadoDic['CREEPERNAMAO']
        self.OBJETIVONATELA = self.estadoDic["OBJETIVONATELA"]

        self.id = self.estadoDic["ID"]
        self.PEGACREEPER = self.estadoDic["PEGACREEPER"]
        self.COR = self.estadoDic["COR"]

    def roda_todo_frame(self, imagem):       
        atraso = 1.5E9
        now = rospy.get_rostime()
        imgtime = imagem.header.stamp
        lag = now-imgtime # calcula o lag
        delay = lag.nsecs

        if delay > atraso and check_delay==True:
            # Esta lógica do delay só precisa ser usada com robo real e rede wifi e serve para descartar imagens antigas
            # print("Descartando por causa do delay do frame:", delay)
            return 
        try:
            temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8") 
            if self.contadorFrame % 3 == 0 or self.PEGACREEPER or self.OBJETIVONATELA:
                self.gray = cv2.cvtColor(temp_image, cv2.COLOR_BGR2GRAY)
                self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.gray, aruco_dict, parameters=parameters)
                self.centro, self.saida_net, self.resultados =  visao_module.processa(temp_image)

                #Exemplo de categoria de resultados (MobileNet)
                # [('chair', 86.965459585189819, (90, 141), (177, 265))]
                
                self.bgr = self.saida_net.copy()
                self.contadorFrame = 0
            else:
                self.bgr = temp_image
                self.ids = None

            hsv = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2HSV)

            self.contadorFrame += 1

            #Aqui é onde se conferem os estados do robô para aplicar as máscaras na pista que manterão ele seguindo a linha amarela:
            
            mask_yellow = rl.segmenta_linha_amarela_bgr(temp_image)
            mask_yellow = rl.morpho_limpa(mask_yellow)

            if self.ESTADO == "DIREITA":
                mask_yellow = rl.maskYellowBloqueiaEsquerda(mask_yellow)

            if self.ESTADO == "ESQUERDA": 
                mask_yellow = rl.maskYellowBloqueiaDireita(mask_yellow)
            
            if self.ESTADO == "DIREITAMAIOR":
                mask_yellow = rl.maskYellowBloqueiaEsquerdaMaior(mask_yellow)

            if  self.COR == "orange":
                maskLaranja, self.media, self.centro, maior_area = cM.identifica_cor_laranja(temp_image)

            if  self.COR == "blue":
                maskAzul, self.media, self.centro, maior_area = cM.identifica_cor_azul(temp_image)
                
            if  self.COR == "green":
                maskVerde, self.media, self.centro, maior_area = cM.identifica_cor_verde(temp_image)

            output, self.coef_angular, self.x_linha = rl.ajuste_linear_grafico_x_fy(mask_yellow)

            
            if self.ids is not None:
                if self.id in self.ids:
                    index = list(self.ids).index(self.id)
                    ret = aruco.estimatePoseSingleMarkers(self.corners[index], self.marker_size, self.camera_matrix, self.camera_distortion)
                else:
                    ret = aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, self.camera_matrix, self.camera_distortion)
                #-- ret = [rvec, tvec, ?]
                #-- rvec = [[rvec_1], [rvec_2], ...] vetor de rotação
                #-- tvec = [[tvec_1], [tvec_2], ...] vetor de translação
                self.rvec, self.tvec = ret[0][0,0,:], ret[1][0,0,:]
                

                #-- Desenha um retanculo e exibe Id do marker encontrado
                aruco.drawDetectedMarkers(temp_image, self.corners, self.ids) 
                aruco.drawAxis(self.bgr, self.camera_matrix, self.camera_distortion, self.rvec, self.tvec, 1)

                #-- Print tvec vetor de tanslação em x y z
                str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(self.tvec[0], self.tvec[1], self.tvec[2])
                print(str_position)
                cv2.putText(self.bgr, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

                ##############----- Referencia dos Eixos------###########################
                # Linha referencia em X
                cv2.line(self.bgr, (int(self.bgr.shape[1]/2),int(self.bgr.shape[0]/2)), ((int(self.bgr.shape[1]/2) + 50),(int(self.bgr.shape[0]/2))), (0,0,255), 5) 
                # Linha referencia em Y
                cv2.line(self.bgr, (int(self.bgr.shape[1]/2),int(self.bgr.shape[0]/2)), (int(self.bgr.shape[1]/2),int((self.bgr.shape[0]/2 + 50))), (0,255,0), 5) 	
                
                #####################---- Distancia Euclidiana ----#####################
                # Calcula a distancia usando apenas a matriz tvec, matriz de tanslação
                # Pode usar qualquer uma das duas formas
                distance = np.sqrt(self.tvec[0]**2 + self.tvec[1]**2 + self.tvec[2]**2)
                self.distancenp = np.linalg.norm(self.tvec)

                #-- Print distance
                str_dist = "Dist aruco=%4.0f  dis.np=%4.0f"%(distance, self.distancenp)
                print(str_dist)
                cv2.putText(self.bgr, str_dist, (0, 15), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

                #####################---- Distancia pelo foco ----#####################
                #https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
                
                # raspicam v2 focal legth 
                FOCAL_LENGTH = 3.6 #3.04
                # pixel por unidade de medida
                m = (self.camera_matrix[0][0]/FOCAL_LENGTH + self.camera_matrix[1][1]/FOCAL_LENGTH)/2
                # corners[0][0][0][0] = [ID][plano?][pos_corner(sentido horario)][0=valor_pos_x, 1=valor_pos_y]	
                pixel_length1 = math.sqrt(math.pow(self.corners[0][0][0][0] - self.corners[0][0][1][0], 2) + math.pow(self.corners[0][0][0][1] - self.corners[0][0][1][1], 2))
                pixel_length2 = math.sqrt(math.pow(self.corners[0][0][2][0] - self.corners[0][0][3][0], 2) + math.pow(self.corners[0][0][2][1] - self.corners[0][0][3][1], 2))
                pixlength = (pixel_length1+pixel_length2)/2
                dist = self.marker_size * FOCAL_LENGTH / (pixlength/m)
                
                #-- Print distancia focal
                str_distfocal = "Dist focal=%4.0f"%(dist)
                print(str_distfocal)
                cv2.putText(self.bgr, str_distfocal, (0, 30), font, 1, (0, 255, 0), 1, cv2.LINE_AA)	


                ####################--------- desenha o cubo -----------#########################
                # https://github.com/RaviJoshii/3DModeler/blob/eb7ca48fa06ca85fcf5c5ec9dc4b562ce9a22a76/opencv/program/detect.py			
                m = self.marker_size/2
                pts = np.float32([[-m,m,m], [-m,-m,m], [m,-m,m], [m,m,m],[-m,m,0], [-m,-m,0], [m,-m,0], [m,m,0]])
                imgpts, _ = cv2.projectPoints(pts, self.rvec, self.tvec, self.camera_matrix, self.camera_distortion)
                imgpts = np.int32(imgpts).reshape(-1,2)
                bgr = cv2.drawContours(self.bgr, [imgpts[:4]],-1,(0,0,255),4)
                for i,j in zip(range(4),range(4,8)): bgr = cv2.line(bgr, tuple(imgpts[i]), tuple(imgpts[j]),(0,0,255),4);
                bgr = cv2.drawContours(bgr, [imgpts[4:]],-1,(0,0,255),4)

            # Desnecessário - Hough e MobileNet já abrem janelas
            cv2.imshow("bgr", self.bgr)
            cv2.imshow("Output", output)
            cv2.waitKey(1)
            
            self.dicCamera = {
                "coef_angular": self.coef_angular,
                "x_linha": self.x_linha,
                "ids": self.ids,
                "media": self.media,
                "centro": self.centro,
                "distancianp": self.distancenp,
                "resultados": self.resultados,
                "rvec": self.rvec
            }

            self.jsonDic = json.dumps(self.dicCamera, cls=MyEncoder)
            self.pubCamera.publish(self.jsonDic)

        except CvBridgeError as e:
            print('ex', e)


def main(args):

  ''' FUNCAO QUE INICIALIZA CAMERA'''

  camera = Camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
