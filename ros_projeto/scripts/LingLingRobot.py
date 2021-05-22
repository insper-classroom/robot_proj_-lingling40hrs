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


class Lingling:
    def __init__(self, cor, id, estacao):
        self.bridge = CvBridge()
        self.estado = LINHA
        rospy.init_node("cor")

        self.topico_imagem = "/camera/image/compressed"
        self.recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
        self.ref_odom = rospy.Subscriber("/odom", Odometry, recebe_odometria)

        self.v = 0
        self.w = 0

        self.missao()


    def scaneou(self, dado):
        #print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
        #print("Leituras:")
        #print(np.array(dado.ranges).round(decimals=2))
        global laserDadoSairCentro
        global laserDadoCreeper
        global laserDadoFrente
        self.laserDadoFrente = dado.ranges[0]
        self.laserDadoSairCentro = dado.ranges[90]
        self.laser0to15 = np.min(dado.ranges[0:45])
        self.laser345to360 = np.min(dado.ranges[315:360])
        listaMinimosLaser = [laser0to15, laser345to360]
        self.laserDadoCreeper = np.min(listaMinimosLaser)
        print(f'\n\n\n\n\n {laserDadoCreeper}')    
        #print("Intensities")
        #print(np.array(dado.intensities).round(decimals=2))

    def recebe_odometria(self,data):
        global x
        global y
        global contador
        global angle_z

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        self.radianos = transformations.euler_from_quaternion(lista)
        self.angle_z = radianos[2]
        self.angulos = np.degrees(radianos)    

        if contador % pula == 0:
            print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
            print(angle_z)
        self.contador = contador + 1

    def angulo(self, angle_z):
        self.angle_deg = math.degrees(angle_z)
        if self.angle_deg < 0:
            self.angle_deg = self.angle_deg + 360.00
        return self.angle_deg

    def andar(self, coef_angular, x_linha):
        meio_linha = (self.x_linha[0] + self.x_linha[1])/2
        erro_x = meio_linha - 360
        self.v = 0.4- abs(erro_x)/900 - 0.16/self.laserDadoFrente
        erro_coefang = self.coef_angular
        self.w = (-erro_x/800) + (erro_coefang/160) # Valor bom para retas em erro_x = 1200 e erro_coefang/160 
        print('VELOCIDADE ANGULAR:', w)

        return v,w

    def giro90 (self, angulo_local, angulo_fin):
        giroCompleto = False
        angulo_init = self.angulo_fin + 90
        dif = abs(angulo_local- angulo_fin)
        if angulo_init < 0:
            angulo_init += 360
        self.v = 0
        self.w = 0
        if giroCompleto == False:
            if angulo_init < 90:  #se o inicial for > 270, significa que o angulo final será positivo, e que assim o robô virar para direita
                if dif <= 5:
                    self.w = 0
                    giroCompleto = True
                    return self.v, self.w, giroCompleto
                self.w = -0.1
            else:
                if dif <= 5: #virar para esquerda, anti-horário, pois os ângulos serão negativos
                    self.w = 0
                    giroCompleto = True
                    return self.v, self.w, giroCompleto
                self.w = 0.1
        return self.v, -self.w, giroCompleto
                