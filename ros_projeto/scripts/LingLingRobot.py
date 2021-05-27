#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
from pickle import TRUE
import rospy
from cv_bridge import CvBridge, CvBridgeError

class Lingling:
    
    def __init__(self, cor, id, estacao):
        rospy.init_node("robô")
        rospy.Publisher        
        self.cor = cor 
        self.id = id 
        self.estacao = estacao
        self.estadoDic = {
                "ESTADO" : "LINHA",
                "ESTACAO" : "",
                "ANDAR" : True,
                "CREEPERNAMAO" : False,
                "ESTADONATELA" : False
        }
        self.ESTADO = self.estadoDic["ESTADO"]

#------------------------------Estados do robô---------------------------------

        # self.LINHA = 0
        # self.CIRCUNF = 1
        # self.DIREITA = 2
        # self.DIREITAMAIOR = 8
        # self.ESQUERDA = 3
        
        # self.GIRO90 = 4
        # self.GIRO180 = 5

        # self.CAVALO = 6
        # self.CACHORRO = 7
        # self.GIROCIRCUNF = 9
        

#-----------------------------Estados de estações-----------------------------

        # self.ESTACAO = 50
        # self.EST_BIF = 51
        # self.EST_CAV = 52
        # self.EST_DOG = 53
        # self.EST_CIR = 54

#------------------------------Flags Booleanas--------------------------------

        self.ESTACAONATELA = False
        self.CREEPERNAMAO = False
        self.tempoCreeper = True                

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            print(self.ESTADO)
            if self.estadoDic["ANDAR"]:
                    self.estadoDic["ESTADO"]()
            r.sleep()

    def LINHA(self):
        