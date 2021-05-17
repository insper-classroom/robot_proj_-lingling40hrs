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

#--- Define Tag de teste
id_to_find  = 200
marker_size  = 20 #- [cm]
#id_to_find  = 22
#marker_size  = 3 #- [cm]
# 


#--- Get the camera calibration path
calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

scan_dist = 0



bridge = CvBridge()

cv_image = None
media = []
centro = []
x = []
ids=[]
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0
x_linha = [0,0]
coef_angular = 0
laserDadoSairCentro = 10.0
laserDadoFrente = 10.0
laserDadoCreeper = 10.0
distancenp = 0
contador = 0
pula = 50
angle_z = 0.0
angulo_desejado = 1000
tempoCreeper = True

# aqui estão todos os estados usados pelo robô.
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
CREEPERNAMAO = 11

ESTADO = LINHA

COR = None
LARANJA = 20
VERDE = 21
AZUL = 22

ESTACAO = 0
EST_BIF = 51
EST_CAV = 52
EST_DOG = 53
EST_CIR = 54

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def scaneou(dado):
    #print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    #print("Leituras:")
    #print(np.array(dado.ranges).round(decimals=2))
    global laserDadoSairCentro
    global laserDadoCreeper
    global laserDadoFrente
    laserDadoFrente = dado.ranges[0]
    laserDadoSairCentro = dado.ranges[90]
    laser0to15 = np.min(dado.ranges[0:45])
    laser345to360 = np.min(dado.ranges[315:360])
    listaMinimosLaser = [laser0to15, laser345to360]
    laserDadoCreeper = np.min(listaMinimosLaser)
    print(f'\n\n\n\n\n {laserDadoCreeper}')    
    #print("Intensities")
    #print(np.array(dado.intensities).round(decimals=2))

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

def angulo (angle_z):
    angle_deg = math.degrees(angle_z)
    if angle_deg < 0:
        angle_deg = angle_deg + 360.00
    return angle_deg

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global coef_angular
    global x_linha
    global ids
    global distancenp

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        # Esta logica do delay so' precisa ser usada com robo real e rede wifi 
        # serve para descartar imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, saida_net, resultados =  visao_module.processa(temp_image) 
        cv_image = saida_net.copy()
        bgr = cv_image.copy()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
              
        for r in range(len(resultados)):
            
            # print(r) - print feito para documentar e entender
            # o resultado
            pass

        mask_yellow = rl.segmenta_linha_amarela_bgr(bgr)
        mask_yellow = rl.morpho_limpa(mask_yellow)

        if ESTADO == DIREITA:
            mask_yellow = rl.maskYellowBloqueiaEsquerda(mask_yellow)

        if ESTADO == ESQUERDA: 
            mask_yellow = rl.maskYellowBloqueiaDireita(mask_yellow)
        
        if ESTADO == DIREITAMAIOR:
            mask_yellow = rl.maskYellowBloqueiaEsquerdaMaior(mask_yellow)

        if  COR == LARANJA:
            maskLaranja, media, centro, maior_area = cM.identifica_cor_laranja(temp_image)
            cv2.imshow("Laranja", maskLaranja)

        if  COR == AZUL:
            maskAzul, media, centro, maior_area = cM.identifica_cor_azul(temp_image)
            cv2.imshow("Azul", maskAzul)
            
        if  COR == VERDE:
            maskLaranja, media, centro, maior_area = cM.identifica_cor_verde(temp_image)
            cv2.imshow("Verde", maskVerde)

        output, coef_angular, x_linha = rl.ajuste_linear_grafico_x_fy(mask_yellow)

        if ids is not None:
            #-- ret = [rvec, tvec, ?]
            #-- rvec = [[rvec_1], [rvec_2], ...] vetor de rotação
            #-- tvec = [[tvec_1], [tvec_2], ...] vetor de translação
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Desenha um retanculo e exibe Id do marker encontrado
            aruco.drawDetectedMarkers(cv_image, corners, ids) 
            aruco.drawAxis(cv_image, camera_matrix, camera_distortion, rvec, tvec, 1)

            #-- Print tvec vetor de tanslação em x y z
            str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            print(str_position)
            cv2.putText(cv_image, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

            ##############----- Referencia dos Eixos------###########################
            # Linha referencia em X
            cv2.line(cv_image, (int(cv_image.shape[1]/2),int(cv_image.shape[0]/2)), ((int(cv_image.shape[1]/2) + 50),(int(cv_image.shape[0]/2))), (0,0,255), 5) 
            # Linha referencia em Y
            cv2.line(cv_image, (int(cv_image.shape[1]/2),int(cv_image.shape[0]/2)), (int(cv_image.shape[1]/2),int((cv_image.shape[0]/2 + 50))), (0,255,0), 5) 	
            
            #####################---- Distancia Euclidiana ----#####################
            # Calcula a distancia usando apenas a matriz tvec, matriz de tanslação
            # Pode usar qualquer uma das duas formas
            distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
            distancenp = np.linalg.norm(tvec)

            #-- Print distance
            str_dist = "Dist aruco=%4.0f  dis.np=%4.0f"%(distance, distancenp)
            print(str_dist)
            cv2.putText(cv_image, str_dist, (0, 15), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

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
            cv2.putText(cv_image, str_distfocal, (0, 30), font, 1, (0, 255, 0), 1, cv2.LINE_AA)	


            ####################--------- desenha o cubo -----------#########################
            # https://github.com/RaviJoshii/3DModeler/blob/eb7ca48fa06ca85fcf5c5ec9dc4b562ce9a22a76/opencv/program/detect.py			
            m = marker_size/2
            pts = np.float32([[-m,m,m], [-m,-m,m], [m,-m,m], [m,m,m],[-m,m,0], [-m,-m,0], [m,-m,0], [m,m,0]])
            imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, camera_distortion)
            imgpts = np.int32(imgpts).reshape(-1,2)
            cv_image = cv2.drawContours(cv_image, [imgpts[:4]],-1,(0,0,255),4)
            for i,j in zip(range(4),range(4,8)): cv_image = cv2.line(cv_image, tuple(imgpts[i]), tuple(imgpts[j]),(0,0,255),4);
            cv_image = cv2.drawContours(cv_image, [imgpts[4:]],-1,(0,0,255),4)

        # Desnecessário - Hough e MobileNet já abrem janelas
        cv2.imshow("cv_image", cv_image)
        cv2.imshow("Output", output)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
        

def andar(coef_angular, x_linha):
    meio_linha = (x_linha[0] + x_linha[1])/2
    erro_x = meio_linha - 360
    v = 0.4- abs(erro_x)/900 - 0.16/laserDadoFrente
    erro_coefang = coef_angular
    w = (-erro_x/800) + (erro_coefang/160) # Valor bom para retas em erro_x = 1200 e erro_coefang/160 
    print('VELOCIDADE ANGULAR:', w)

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
    angulo_init = angulo_fin - 180
    dif = abs(angulo_local- angulo_fin)
    if angulo_init < 0:
        angulo_init += 360
    v = 0
    w = 0
    if giroCompleto == False:
        if angulo_init > 180:
            if dif <= 5:
                w = 0
                giroCompleto = True
                return v, w, giroCompleto
            w = -0.1
        else:
            if dif <= 5:
                w = 0
                giroCompleto = True
                return v, w, giroCompleto
            w = 0.1
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
        para fazer a garra funfar
    '''


    ''' O QUE FALTA?
        CONCEITO C:
        Pega o creeper da cor e ID corretos com a garra e volta para a pista.No retorno à pista o grupo de alunos não precisa gravar vídeo comprobatório por muito tempo. 
        Apenas o suficiente para demonstrar que o robô encontrou a pista e voltou a executar o código de seguir. GARRA

        CONCEITO B:
        Itens do conceito B + um uso de classes e objetos Python
        Só pode ter sleep dentro do while principal.
        Pegar o creeper da cor certa, com o ID certo, e deixar na base certa   CLASSES

        CONCEITO A:
        Fazer um controle proporcional ou PD para manter o robô na pista e fazer funcionar rápido baseado no ângulo de visão da pista, mais ou menos como neste exemplo
        Usar ARUCO em modo 3D
        Encontrar os creepers que se encontram fora da pista usando mapeamento ( https://github.com/Insper/404/blob/master/tutoriais/robotica/navigation_gazebo_simulador.md)'''



    ombro_publisher = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
    garra_publisher = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25
    ANDAR = True
    PEGARCREEPER = False
    naoChegou80Cm = True
    estacaoCentral = False
    mediax = 0

    try:
        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        cor = 'orange' # input("Qual é a cor do creeper?: ")
        id = 11   #input("Qual é o id do creeper?")
        estacao = 'dog'  # input ("Qual é a estação que você quer levar o creeper?")

        goal = (cor, id, estacao)
        
        while not rospy.is_shutdown():
            #for r in resultados:
                #print(r)
                
            #goal1 = ("blue", 22, "dog")
            if ANDAR:
                v = 0
                w = 0 
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

                #IR PARA ESTAÇÃOOOO TENTAR DEPOIS 
                elif ESTADO == CREEPERNAMAO:
                
                    for r in resultados:
                        print(r)
                        print("gatoC", estacaoCentral)
                        print("naoChegou", naoChegou80Cm)
                        if r[0] == estacao:
                            mediax = abs((r[2][0] + r[3][0])/2)
                    #velocidade_saida.publish(vel)
                    if laserDadoFrente >= 0.8 and naoChegou80Cm == True:
                        v,w = andar(coef_angular, x_linha)
                    elif naoChegou80Cm == False and estacaoCentral == False:
                        v = 0.1
                    else:   
                        if laserDadoFrente == 0: #no começo do Frame o laser dado == 0
                            naoChegou80Cm = True
                        else:
                            v = 0
                            w = 0
                            naoChegou80Cm = False
                    if len(centro) != 0 and mediax != 0:
                        if abs(centro[0] - mediax) <= 10:
                            estacaoCentral = True
                            if laserDadoFrente >= 1:
                                v = 0
                            elif laserDadoFrente < 1:
                                v = -0.1
                    
                
                if ids is not None:        
                    print("CHEQUEI")
                    if laserDadoCreeper <= 1.5 and media[0]!=0 and centro[0] !=0 and id in ids and distancenp <= 700:
                        print("ENTREI")
                        PEGARCREEPER = True
                        ANDAR = False

            elif PEGARCREEPER:
                if laserDadoCreeper <= 0.17:
                    if tempoCreeper:
                        tempo_inicial = rospy.get_time()
                        tempoCreeper = False
                        
                    w = 0
                    v = 0
                    garra_publisher.publish(-1.0) #aberto
                    ombro_publisher.publish(0.0) #para cimaa: metade
                    tempo_final = rospy.get_time()
                    if tempo_final - tempo_inicial >= 2.5:
                        garra_publisher.publish(0.0) #fechado
                        if tempo_final - tempo_inicial >= 5:
                            ombro_publisher.publish(1.5)
                            ESTADO =  CREEPERNAMAO
                            PEGARCREEPER = False
                            ANDAR = True  #volta a andar 
                          
                else:
                    erro_xcreeper = media[0] - centro[0]
                    w= - (erro_xcreeper/900)
                    v= 0.4 - 0.072/laserDadoCreeper - abs(erro_xcreeper)/1500
                print('\n\n\n\n achemos O CRÉPE')  
                    
                

            if ids is not None:
                if 100 in ids:
                    ESTACAO = EST_BIF
                if 50 in ids:
                    ESTACAO = EST_CAV
                if 150 in ids: 
                    ESTACAO = EST_DOG
                if 200 in ids:
                    ESTACAO = EST_CIR

            if ESTACAO == EST_BIF and laserDadoFrente <= 1.8:
                ESTADO = DIREITA
            elif ESTACAO == EST_CAV and laserDadoFrente <= 1.2:
                ESTADO = CAVALO
            elif ESTACAO == EST_DOG and laserDadoFrente <= 1.2:
                ESTADO = CACHORRO
            elif ESTACAO == EST_CIR and laserDadoFrente <= 0.8:
                ESTADO = GIROCIRCUNF

            
            if cor == 'orange':
                COR = LARANJA
            elif cor == 'blue':
                COR = AZUL
            else:
                COR = VERDE


            #print("Vel lin :{0}".format(v))
            #print("Vel ang: {0}".format(w))
            print (ESTADO)
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


