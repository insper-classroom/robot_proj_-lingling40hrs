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

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module
import regressaoLinear as rl


bridge = CvBridge()

cv_image = None
media = []
centro = []
x = []
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
coef_angular = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global coef_angular
    global x

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
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        cv_image = saida_net.copy()
        bgr = cv_image.copy()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask_yellow = rl.segmenta_linha_amarela_bgr(bgr)
        mask_yellow = rl.morpho_limpa(mask_yellow)
        output, coef_angular, x = rl.ajuste_linear_grafico_x_fy(mask_yellow)
        print(x)

        # Desnecessário - Hough e MobileNet já abrem janelas
        cv2.imshow("cv_image", cv_image)
        cv2.imshow("Output", output)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    try:
        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            v = 0
            w = 0
            if -0.2 <= coef_angular <= 0.2:
                v = 0.1
                w = 0
            elif coef_angular > 0.2:
                if 300 < x[1] < 360:
                    v = 0.05
                    w = 0.05
                elif x[1] > 360:
                    v = 0.05
                    w = -0.07
                else:
                    v = 0.05
                    w = 0.07
            elif coef_angular < -0.2:
                if 300 < x[0] < 360:
                    v = 0.05
                    w = -0.05
                elif x[0] > 480:
                    v = 0.05
                    w = -0.07
                else:
                    v = 0.05
                    w = 0.07
            else:
                v = 0
                w = 0
                
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


