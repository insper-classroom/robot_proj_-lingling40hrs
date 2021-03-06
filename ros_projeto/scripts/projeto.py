#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import Float64



print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module
import regressaoLinear as rl
import corModule as cM
import cv2.aruco as aruco

#---------------------------Parâmetros do aruco--------------------------------

id_to_find  = 200
marker_size  = 20 #[cm]


#Get the camera calibration path

calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')


#Define the aruco dictionar

aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000


#Font for the text in the image

font = cv2.FONT_HERSHEY_PLAIN
scan_dist = 0

bridge = CvBridge()

#-------------------------Variáveis de posicionamento-------------------------

bgr = None
media = []
centro = []
ids=[]
atraso = 1.5E9 #1 segundo e meio, em nanossegundos

#Variavel com a área do maior contorno

area = 0.0

#Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
#Descarta imagens que chegam atrasadas demais

check_delay = False 

resultados = [] #Criação de uma variável global para guardar os resultados vistos

#---------------------------Variáveis da odometria--------------------------

x = 0
y = 0
z = 0 
angle_z = 0.0
contador = 0
pula = 50

#-----------------------------Variáveis do laser---------------------------

coef_angular = 0
laserDadoSairCentro = 10.0
laserDadoFrente = 10.0
laserDadoCreeper = 10.0

#----------------------------Variáveis do Aruco----------------------------

distancenp = 0
id = 0
rvec = 0

#----------------Lista das abcissas da linha da regressão linear---------------

x_linha = [0,0]

#---------------------------------Variáveis miscs------------------------------

angulo_desejado = 1000
contadorFrame = 0

#------------------------------Estados do robô---------------------------------

LINHA = 0
CIRCUNF = 1
DIREITA = 2
ESQUERDA = 3
GIRO90 = 4
GIRO180 = 5
CAVALO = 6
CACHORRO = 7
DIREITAMAIOR = 8
GIROCIRCUNF = 9
ESTADO = LINHA

ESTACAONATELA = False
CREEPERNAMAO = False
tempoCreeper = True

#------------------------------Estados de cores-------------------------------

COR = None
LARANJA = 20
VERDE = 21
AZUL = 22

#-----------------------------Estados de estações-----------------------------

ESTACAO = 0
EST_BIF = 51
EST_CAV = 52
EST_DOG = 53
EST_CIR = 54

#-----------------------------------------------------------------------------------------------

frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()

#----------Função que utiliza o laser para conferir as distâncias de diferentes ângulos---------

def scaneou(dado):
    global laserDadoSairCentro
    global laserDadoCreeper
    global laserDadoFrente
    laserDadoFrente = dado.ranges[0]
    laserDadoSairCentro = dado.ranges[90]
    laser0to15 = np.min(dado.ranges[0:45])
    laser345to360 = np.min(dado.ranges[315:360])
    listaMinimosLaser = [laser0to15, laser345to360]
    laserDadoCreeper = np.min(listaMinimosLaser)    


#----Função responsável por aplicar a odometria e saber a posição do robô e seu ângulo---------

def recebe_odometria(data):
    global x
    global y
    global contador
    global angle_z

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    radianos = transformations.euler_from_quaternion(lista)
    angle_z = radianos[2]
    angulos = np.degrees(radianos)    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
        print(angle_z)
    contador = contador + 1

#--------------------Função que define o ângulo z----------------------

def angulo (angle_z):
    angle_deg = math.degrees(angle_z)
    if angle_deg < 0:
        angle_deg = angle_deg + 360.00
    return angle_deg

#-----------------Função que roda em todo frame------------------------

def roda_todo_frame(imagem):
    print("frame")
    global bgr
    global media
    global centro
    global resultados
    global coef_angular
    global x_linha
    global ids
    global distancenp
    global contadorFrame
    global rvec

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs

    if delay > atraso and check_delay==True:
        # Esta lógica do delay só precisa ser usada com robo real e rede wifi e serve para descartar imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        if contadorFrame % 3 == 0 or PEGARCREEPER or ESTACAONATELA:
            gray = cv2.cvtColor(temp_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            centro, saida_net, resultados =  visao_module.processa(temp_image)

            #Exemplo de categoria de resultados (MobileNet)
            # [('chair', 86.965459585189819, (90, 141), (177, 265))]
            
            bgr = saida_net.copy()
            contadorFrame = 0
        else:
            bgr = temp_image
            ids = None

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        contadorFrame += 1

        #Aqui é onde se conferem os estados do robô para aplicar as máscaras na pista que manterão ele seguindo a linha amarela:
        
        mask_yellow = rl.segmenta_linha_amarela_bgr(temp_image)
        mask_yellow = rl.morpho_limpa(mask_yellow)

        if ESTADO == DIREITA:
            mask_yellow = rl.maskYellowBloqueiaEsquerda(mask_yellow)

        if ESTADO == ESQUERDA: 
            mask_yellow = rl.maskYellowBloqueiaDireita(mask_yellow)
        
        if ESTADO == DIREITAMAIOR:
            mask_yellow = rl.maskYellowBloqueiaEsquerdaMaior(mask_yellow)

        if  COR == LARANJA:
            maskLaranja, media, centro, maior_area = cM.identifica_cor_laranja(temp_image)

        if  COR == AZUL:
            maskAzul, media, centro, maior_area = cM.identifica_cor_azul(temp_image)
            
        if  COR == VERDE:
            maskVerde, media, centro, maior_area = cM.identifica_cor_verde(temp_image)

        output, coef_angular, x_linha = rl.ajuste_linear_grafico_x_fy(mask_yellow)

        
        if ids is not None:
            if id in ids:
                index = list(ids).index(id)
                ret = aruco.estimatePoseSingleMarkers(corners[index], marker_size, camera_matrix, camera_distortion)
            else:
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            #-- ret = [rvec, tvec, ?]
            #-- rvec = [[rvec_1], [rvec_2], ...] vetor de rotação
            #-- tvec = [[tvec_1], [tvec_2], ...] vetor de translação
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            

            #-- Desenha um retanculo e exibe Id do marker encontrado
            aruco.drawDetectedMarkers(temp_image, corners, ids) 
            aruco.drawAxis(bgr, camera_matrix, camera_distortion, rvec, tvec, 1)

            #-- Print tvec vetor de tanslação em x y z
            str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            print(str_position)
            cv2.putText(bgr, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

            ##############----- Referencia dos Eixos------###########################
            # Linha referencia em X
            cv2.line(bgr, (int(bgr.shape[1]/2),int(bgr.shape[0]/2)), ((int(bgr.shape[1]/2) + 50),(int(bgr.shape[0]/2))), (0,0,255), 5) 
            # Linha referencia em Y
            cv2.line(bgr, (int(bgr.shape[1]/2),int(bgr.shape[0]/2)), (int(bgr.shape[1]/2),int((bgr.shape[0]/2 + 50))), (0,255,0), 5) 	
            
            #####################---- Distancia Euclidiana ----#####################
            # Calcula a distancia usando apenas a matriz tvec, matriz de tanslação
            # Pode usar qualquer uma das duas formas
            distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
            distancenp = np.linalg.norm(tvec)

            #-- Print distance
            str_dist = "Dist aruco=%4.0f  dis.np=%4.0f"%(distance, distancenp)
            print(str_dist)
            cv2.putText(bgr, str_dist, (0, 15), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

            #####################---- Distancia pelo foco ----#####################
            #https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
            
            # raspicam v2 focal legth 
            FOCAL_LENGTH = 3.6 #3.04
            # pixel por unidade de medida
            m = (camera_matrix[0][0]/FOCAL_LENGTH + camera_matrix[1][1]/FOCAL_LENGTH)/2
            # corners[0][0][0][0] = [ID][plano?][pos_corner(sentido horario)][0=valor_pos_x, 1=valor_pos_y]	
            pixel_length1 = math.sqrt(math.pow(corners[0][0][0][0] - corners[0][0][1][0], 2) + math.pow(corners[0][0][0][1] - corners[0][0][1][1], 2))
            pixel_length2 = math.sqrt(math.pow(corners[0][0][2][0] - corners[0][0][3][0], 2) + math.pow(corners[0][0][2][1] - corners[0][0][3][1], 2))
            pixlength = (pixel_length1+pixel_length2)/2
            dist = marker_size * FOCAL_LENGTH / (pixlength/m)
            
            #-- Print distancia focal
            str_distfocal = "Dist focal=%4.0f"%(dist)
            print(str_distfocal)
            cv2.putText(bgr, str_distfocal, (0, 30), font, 1, (0, 255, 0), 1, cv2.LINE_AA)	


            ####################--------- desenha o cubo -----------#########################
            # https://github.com/RaviJoshii/3DModeler/blob/eb7ca48fa06ca85fcf5c5ec9dc4b562ce9a22a76/opencv/program/detect.py			
            m = marker_size/2
            pts = np.float32([[-m,m,m], [-m,-m,m], [m,-m,m], [m,m,m],[-m,m,0], [-m,-m,0], [m,-m,0], [m,m,0]])
            imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, camera_distortion)
            imgpts = np.int32(imgpts).reshape(-1,2)
            bgr = cv2.drawContours(bgr, [imgpts[:4]],-1,(0,0,255),4)
            for i,j in zip(range(4),range(4,8)): bgr = cv2.line(bgr, tuple(imgpts[i]), tuple(imgpts[j]),(0,0,255),4);
            bgr = cv2.drawContours(bgr, [imgpts[4:]],-1,(0,0,255),4)

        # Desnecessário - Hough e MobileNet já abrem janelas
        cv2.imshow("bgr", bgr)
        cv2.imshow("Output", output)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
        

def andar(coef_angular, x_linha):
    meio_linha = (x_linha[0] + x_linha[1])/2
    erro_x = meio_linha - 360
    v = 0.4- abs(erro_x)/900 - 0.16/laserDadoFrente
    erro_coefang = coef_angular
    w = (-erro_x/800) + (erro_coefang/160) 

    return v,w

def giro90 (angulo_local, angulo_fin):
    giroCompleto = False
    angulo_init = angulo_fin + 90
    dif = abs(angulo_local- angulo_fin)
    if angulo_init < 0:
        angulo_init += 360
    v = 0
    w = 0
    if giroCompleto == False:
        if angulo_init < 90:  #se o inicial for > 270, significa que o angulo final será positivo, e que assim o robô virar para direita
            if dif <= 5:
                w = 0
                giroCompleto = True
                return v, w, giroCompleto
            w = -0.1
        else:
            if dif <= 5: #virar para esquerda, anti-horário, pois os ângulos serão negativos
                w = 0
                giroCompleto = True
                return v, w, giroCompleto
            w = 0.1
    return v, -w, giroCompleto

def giro180 (angulo_local, angulo_fin):
    giroCompleto = False
    dif = abs(angulo_local- angulo_fin)
    v = 0
    w = 0
    if dif <= 5:
        giroCompleto = True
        return v, w, giroCompleto
    w = -0.2
    
    return v, w, giroCompleto

if __name__=="__main__":
    rospy.init_node("cor")
    topico_imagem = "/camera/image/compressed"
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    '''
        Antes de iniciar o programa é necessário rodar
        roslaunch mybot_description mybot_control2.launch
        para fazer a garra funcionar
    '''


    ''' O QUE FALTA?
        CONCEITO C:
        Pega o creeper da cor e ID corretos com a garra e volta para a pista.No retorno à pista o grupo de alunos não precisa gravar vídeo comprobatório por muito tempo. 
        Apenas o suficiente para demonstrar que o robô encontrou a pista e voltou a executar o código de seguir. GARRA

        CONCEITO B:
        Itens do conceito B + um uso de classes e objetos Python
        Só pode ter sleep dentro do while principal.
        Pegar o creeper da cor certa, com o ID certo, e deixar na base certa   

        CONCEITO A:
        Fazer um controle proporcional ou PD para manter o robô na pista e fazer funcionar rápido baseado no ângulo de visão da pista, mais ou menos como neste exemplo
        Usar ARUCO em modo 3D
        Encontrar os creepers que se encontram fora da pista usando mapeamento ( https://github.com/Insper/404/blob/master/tutoriais/robotica/navigation_gazebo_simulador.md)'''



    ombro_publisher = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
    garra_publisher = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #Conversão do sistema de coordenadas 
    ANDAR = True
    PEGARCREEPER = False
    estacaoNaTela = False

    try:
        '''goal1 = ("blue", 22, "dog")

        goal2 = ("green", 13, "car")

        goal3 = ("orange", 11, "horse")'''

        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        cor = 'orange' # Qual é a cor do creeper?
        id = 11   # Qual o id do creeper?
        estacao = 'horse' # Qual é a estação que você quer levar o creeper?
        
        goal = (cor, id, estacao)
        
        while not rospy.is_shutdown():
            v = 0
            w = 0 
                
            #goal1 = ("blue", 22, "dog")
            if ANDAR:
                
                angulo_local = angulo(angle_z)
                if ESTADO == LINHA:
                    v,w = andar(coef_angular, x_linha)

                if ESTADO == DIREITA:
                    v,w = andar(coef_angular, x_linha)

                elif ESTADO == ESQUERDA:
                    v,w = andar(coef_angular, x_linha)

                elif ESTADO == DIREITAMAIOR:
                    v,w = andar(coef_angular, x_linha)

                elif ESTADO == GIRO90:
                    if angulo_desejado == 1000:
                        angulo_desejado = angulo_local - 90.00
                        if angulo_desejado > 360:
                            angulo_desejado -= 360
                        if angulo_desejado < 0:
                            angulo_desejado += 360
                    v, w, completaGiro = giro90(angulo_local, angulo_desejado)
                    if completaGiro == True:
                        angulo_desejado = 1000
                        ESTADO = LINHA

                elif ESTADO == GIRO180:
                    if angulo_desejado == 1000:
                        angulo_desejado = angulo_local + 180.00
                        if angulo_desejado > 360:
                            angulo_desejado -= 360
                    v, w, completaGiro = giro180(angulo_local, angulo_desejado)
                    if completaGiro == True:
                        angulo_desejado = 1000
                        ESTADO = LINHA

                elif ESTADO == CAVALO:
                    if angulo_desejado == 1000:
                        angulo_desejado = angulo_local + 180.00
                        if angulo_desejado > 360:
                            angulo_desejado -= 360
                    v, w, completaGiro = giro180(angulo_local, angulo_desejado)
                    if completaGiro == True:
                        angulo_desejado = 1000
                        ESTADO = DIREITAMAIOR

                elif ESTADO == CACHORRO:
                    if angulo_desejado == 1000:
                        angulo_desejado = angulo_local + 180.00
                        if angulo_desejado > 360:
                            angulo_desejado -= 360
                    v, w, completaGiro = giro180(angulo_local, angulo_desejado)
                    if completaGiro == True:
                        angulo_desejado = 1000
                        ESTADO = DIREITA

                elif ESTADO == CIRCUNF:
                    v,w = andar(coef_angular, x_linha)
                    if laserDadoSairCentro <= 0.5 and angulo_local <= 280:
                        ESTADO = GIRO90

                elif ESTADO == GIROCIRCUNF:
                    if angulo_desejado == 1000:
                        angulo_desejado = angulo_local - 45.00
                        if angulo_desejado > 360:
                            angulo_desejado -= 360
                        if angulo_desejado < 0:
                            angulo_desejado += 360
                    v, w, completaGiro = giro90(angulo_local, angulo_desejado)
                    if completaGiro == True:
                        angulo_desejado = 1000
                        ESTADO = CIRCUNF

                if ids is not None:        
                    if laserDadoCreeper <= 1.5 and media[0]!=0 and centro[0]!=0 and id in ids and distancenp <= 800:
                        PEGARCREEPER = True
                        ANDAR = False        
         
                if ids is not None:
                    if 100 in ids and distancenp <= 300:
                        ESTACAO = EST_BIF
                    if 50 in ids and distancenp  <= 300:
                        ESTACAO = EST_CAV
                    if 150 in ids and distancenp <= 300: 
                        ESTACAO = EST_DOG
                    if 200 in ids and distancenp <= 500:
                        ESTACAO = EST_CIR

                    if ESTACAO == EST_BIF and laserDadoFrente <= 1.8:
                        ESTADO = DIREITA
                    elif ESTACAO == EST_CAV and laserDadoFrente <= 1.2:
                        ESTADO = CAVALO
                    elif ESTACAO == EST_DOG and laserDadoFrente <= 1.2:
                        ESTADO = CACHORRO
                    elif ESTACAO == EST_CIR and laserDadoFrente <= 1.2:
                        ESTADO = GIROCIRCUNF

            elif PEGARCREEPER:

                erro_xcreeper = media[0] - centro[0]
                if laserDadoCreeper <= 0.16:
                    if tempoCreeper:
                        tempo_inicial = rospy.get_time()
                        tempoCreeper = False
                        
                    w = 0
                    v = 0
                    garra_publisher.publish(-1.0) #aberto
                    ombro_publisher.publish(0.0) #para cima: metade
                    tempo_final = rospy.get_time()
                    if tempo_final - tempo_inicial >= 2.5:
                        garra_publisher.publish(0.0) #fechado
                        if tempo_final - tempo_inicial >= 5:
                            ombro_publisher.publish(1.5)
                            CREEPERNAMAO = True
                            PEGARCREEPER = False
                            ANDAR = True  #volta a andar

                
                elif 0.16 < laserDadoCreeper <= 0.3:
                    w = (rvec[2]/10) - (erro_xcreeper/900)
                    v = 0.2 - 0.032/laserDadoCreeper     
                else:
                    w = (-rvec[2]/10) - (erro_xcreeper/900)
                    v= 0.4 - 0.072/laserDadoCreeper - abs(erro_xcreeper)/1500

            
            if CREEPERNAMAO:
                for r in resultados:
                    if r[0] == estacao and r[1] >= 95:
                        mediaxMobileNet = abs((r[2][0] + r[3][0])/2)
                        if not ESTACAONATELA:
                            ESTACAONATELA = True
                            ANDAR = False
                            tempoCreeper = True

            if ESTACAONATELA:
                print("ENTREI", laserDadoCreeper)
                erro_xEstacao = mediaxMobileNet - centro[0]
                if laserDadoFrente <= 0.3:
                    print("PAREI")
                    if tempoCreeper:
                        tempo_inicial = rospy.get_time()
                        tempoCreeper = False      
                        w = 0
                        v = 0

                    ombro_publisher.publish(0.0) #para cima: metade
                    tempo_final = rospy.get_time()
                    if tempo_final - tempo_inicial >= 2.5:
                        garra_publisher.publish(-1.0) #aberto
                        ANDAR = False
                        ESTACAONATELA = False
                        CREEPERNAMAO = False

                elif 0.3 < laserDadoFrente <= 0.45:
                    w = - (erro_xEstacao/900)
                    v = 0.2 - 0.04/laserDadoCreeper
                else:
                    w= - (erro_xEstacao/900)
                    v= 0.4 - 0.072/laserDadoCreeper - abs(erro_xEstacao)/1500                       
                    
            if cor == 'orange':
                COR = LARANJA
            if cor == 'blue':
                COR = AZUL
            if cor == 'green':
                COR = VERDE

            print (ESTADO)
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


