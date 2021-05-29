#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import json
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from tf import transformations
from tf import TransformerROS
import tf2_ros
import moduloMovimento as mv

class Lingling:
    
    def __init__(self, cor, id, objetivo):
        self.pubEstados = rospy.Publisher("Estados", String, queue_size=10)
        rospy.init_node("robô") 

        self.cor = cor 
        self.id = [id] 
        self.objetivo = objetivo

        self.estadoDic = {
                'ESTADO' : "LINHA",
                'ESTACAO' : "",
                'ANDAR' : True,
                'PEGACREEPER' : False,
                'CREEPERNAMAO' : False,
                'OBJETIVONATELA' : False
        }
        self.estadoDic["COR"] = self.cor
        self.estadoDic["ID"] = self.id
        self.estadoDic["OBJETIVO"] = self.objetivo

        self.ESTADO = self.estadoDic['ESTADO']
        self.ESTACAO = self.estadoDic['ESTACAO']
        self.ANDAR = self.estadoDic['ANDAR']
        self.CREEPERNAMAO = self.estadoDic['CREEPERNAMAO']
        self.OBJETIVONATELA = self.estadoDic["OBJETIVONATELA"]
        self.PEGACREEPER = self.estadoDic["PEGACREEPER"]

        self.angulo_local = 0.0
        self.angulo_desejado = 1000

        self.coef_angular = 0
        self.x_linha = [0,0]
        self.ids = []
        self.media = [0, 0]
        self.centro = [0,0]
        self.distancianp = 0
        self.resultados = []
        self.rvec = 0

        self.laserDadoFrente = 10
        self.laserDadoSairCentro = 10
        self.laserDadoCreeper = 10
        self.tempoCreeper = True   

        self.subCamera = rospy.Subscriber("Camera",String,self.callbackCamera)
        self.subLaser = rospy.Subscriber("DadosLaser", String,self.callbackLaser)
        self.subOdo = rospy.Subscriber("Angulo",String,self.callbackOdo)

        ombro_publisher = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        tf_buffer = tf2_ros.Buffer()
        garra_publisher = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        tfl = tf2_ros.TransformListener(tf_buffer) #Conversão do sistema de coordenadas 
        self.ANDAR = True
        self.PEGACREEPER = False
        self.OBJETIVONATELA = False

        r_rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            v = 0
            w = 0 
                
            #goal1 = ("blue", 22, "dog")
            if self.ANDAR:
                
                if self.ESTADO == "LINHA":
                    v,w = mv.andar(self.coef_angular, self.x_linha, self.laserDadoFrente)

                if self.ESTADO == "DIREITA":
                    v,w = mv.andar(self.coef_angular, self.x_linha, self.laserDadoFrente)

                elif self.ESTADO == "ESQUERDA":
                    v,w = mv.andar(self.coef_angular, self.x_linha, self.laserDadoFrente)

                elif self.ESTADO == "DIREITAMAIOR":
                    v,w = mv.andar(self.coef_angular, self.x_linha, self.laserDadoFrente)

                elif self.ESTADO == "GIRO90":
                    if self.angulo_desejado == 1000:
                        self.angulo_desejado = self.angulo_local - 90.00
                        if self.angulo_desejado > 360:
                            self.angulo_desejado -= 360
                        if self.angulo_desejado < 0:
                            self.angulo_desejado += 360
                    v, w, completaGiro = mv.giro90(self.angulo_local, self.angulo_desejado)
                    if completaGiro == True:
                        self.angulo_desejado = 1000
                        self.ESTADO = "LINHA"

                elif self.ESTADO == "GIRO180":
                    if self.angulo_desejado == 1000:
                        self.angulo_desejado = self.angulo_local + 180.00
                        if self.angulo_desejado > 360:
                            self.angulo_desejado -= 360
                    v, w, completaGiro = mv.giro180(self.angulo_local, self.angulo_desejado)
                    if completaGiro == True:
                        self.angulo_desejado = 1000
                        self.ESTADO = "LINHA"

                elif self.ESTADO == "CAVALO":
                    if self.angulo_desejado == 1000:
                        self.angulo_desejado = self.angulo_local + 180.00
                        if self.angulo_desejado > 360:
                            self.angulo_desejado -= 360
                    v, w, completaGiro = mv.giro180(self.angulo_local, self.angulo_desejado)
                    if completaGiro == True:
                        self.angulo_desejado = 1000
                        self.ESTADO = "DIREITAMAIOR"

                elif self.ESTADO == "CACHORRO":
                    if self.angulo_desejado == 1000:
                        self.angulo_desejado = self.angulo_local + 180.00
                        if self.angulo_desejado > 360:
                            self.angulo_desejado -= 360
                    v, w, completaGiro = mv.giro180(self.angulo_local, self.angulo_desejado)
                    if completaGiro == True:
                        self.angulo_desejado = 1000
                        self.ESTADO = "DIREITA"

                elif self.ESTADO == "CIRCUNF":
                    v,w = mv.andar(self.coef_angular, self.x_linha, self.laserDadoFrente)
                    if self.laserDadoSairCentro <= 0.5 and self.angulo_local <= 280:
                        self.ESTADO = "GIRO90"

                elif self.ESTADO == "GIROCIRCUNF":
                    if self.angulo_desejado == 1000:
                        self.angulo_desejado = self.angulo_local - 45.00
                        if self.angulo_desejado > 360:
                            self.angulo_desejado -= 360
                        if self.angulo_desejado < 0:
                            self.angulo_desejado += 360
                    v, w, completaGiro = mv.giro90(self.angulo_local, self.angulo_desejado)
                    if completaGiro == True:
                        self.angulo_desejado = 1000
                        self.ESTADO = "CIRCUNF"

                if self.ids is not None and self.media is not None:
                    if self.laserDadoCreeper <= 1.5 and self.media[0]!=0 and self.centro[0]!=0 and self.id in self.ids and self.distancianp <= 800:
                        self.PEGACREEPER = True
                        self.ANDAR = False        
         
                if self.ids is not None:
                    if [100] in self.ids and self.distancianp <= 300:
                        self.ESTACAO = "EST_BIF"
                    if [50] in self.ids and self.distancianp  <= 300:
                        self.ESTACAO = "EST_CAV"
                    if [150] in self.ids and self.distancianp <= 300: 
                        self.ESTACAO = "EST_DOG"
                    if [200] in self.ids and self.distancianp <= 500:
                        self.ESTACAO = "EST_CIR"

                    if self.ESTACAO == "EST_BIF" and self.laserDadoFrente <= 1.8:
                        self.ESTADO = "DIREITA"
                    elif self.ESTACAO == "EST_CAV" and self.laserDadoFrente <= 1.2:
                        self.ESTADO = "CAVALO"
                    elif self.ESTACAO == "EST_DOG" and self.laserDadoFrente <= 1.2:
                        self.ESTADO = "CACHORRO"
                    elif self.ESTACAO == "EST_CIR" and self.laserDadoFrente <= 1.2:
                        self.ESTADO = "GIROCIRCUNF"

            elif self.PEGACREEPER:

                self.erro_xcreeper = self.media[0] - self.centro[0]
                if self.laserDadoCreeper <= 0.16:
                    if self.tempoCreeper:
                        self.tempo_inicial = rospy.get_time()
                        self.tempoCreeper = False
                        
                    w = 0
                    v = 0
                    garra_publisher.publish(-1.0) #aberto
                    ombro_publisher.publish(0.0) #para cima: metade
                    self.tempo_final = rospy.get_time()
                    if self.tempo_final - self.tempo_inicial >= 2.5:
                        garra_publisher.publish(0.0) #fechado
                        if self.tempo_final - self.tempo_inicial >= 5:
                            ombro_publisher.publish(1.5)
                            self.CREEPERNAMAO = True
                            self.PEGACREEPER = False
                            self.ANDAR = True  #volta a andar

                
                elif 0.16 < self.laserDadoCreeper <= 0.3:
                    w = (self.rvec[2]/10) - (self.erro_xcreeper/900)
                    v = 0.2 - 0.032/self.laserDadoCreeper     
                else:
                    w = (-self.rvec[2]/10) - (self.erro_xcreeper/900)
                    v= 0.4 - 0.072/self.laserDadoCreeper - abs(self.erro_xcreeper)/1500

            
            if self.CREEPERNAMAO:
                for r in self.resultados:
                    if r[0] == self.objetivo and r[1] >= 95:
                        print("-------------------------TO INO PRA ESTACAO-----------------")
                        self.mediaxMobileNet = abs((r[2][0] + r[3][0])/2)
                        if not self.OBJETIVONATELA:
                            self.OBJETIVONATELA = True
                            self.ANDAR = False
                            self.tempoCreeper = True

            if self.OBJETIVONATELA:
                print("ENTREI", self.laserDadoCreeper)
                self.erro_xEstacao = self.mediaxMobileNet - self.centro[0]
                if self.laserDadoFrente <= 0.3:
                    print("PAREI")
                    if self.tempoCreeper:
                        self.tempo_inicial = rospy.get_time()
                        self.tempoCreeper = False      
                        w = 0
                        v = 0

                    ombro_publisher.publish(0.0) #para cima: metade
                    self.tempo_final = rospy.get_time()
                    if self.tempo_final - self.tempo_inicial >= 2.5:
                        garra_publisher.publish(-1.0) #aberto
                        self.ANDAR = False
                        self.OBJETIVONATELA = False
                        self.CREEPERNAMAO = False

                elif 0.3 < self.laserDadoFrente <= 0.45:
                    w = - (self.erro_xEstacao/900)
                    v = 0.2 - 0.04/self.laserDadoCreeper
                else:
                    w= - (self.erro_xEstacao/900)
                    v= 0.4 - 0.072/self.laserDadoCreeper - abs(self.erro_xEstacao)/1500                       


            self.estadoDic['ESTADO'] = self.ESTADO
            self.estadoDic['ESTACAO'] = self.ESTACAO
            self.estadoDic['ANDAR'] = self.ANDAR
            self.estadoDic['CREEPERNAMAO'] = self.CREEPERNAMAO
            self.estadoDic['PEGACREEPER'] = self.PEGACREEPER
            self.estadoDic['OBJETIVONATELA'] = self.OBJETIVONATELA

            self.jsonDic = json.dumps(self.estadoDic)
            self.pubEstados.publish(self.jsonDic)


            print (self.ESTADO)
            print(self.ESTACAO)
            print(f'OBJETIVO NA TELA: {self.OBJETIVONATELA}')
            print(f'CREEPER NA MAO: {self.CREEPERNAMAO}\n\n\n')
            #print(f'\n\ndistancai:{self.distancianp} & laser:{self.laserDadoFrente}\n\n')
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            self.velocidade_saida.publish(vel)
            r_rate.sleep()


        
    def callbackCamera(self, dicCamera):
        # conversao json (lembrar do .data)
        self.dicCamera = json.loads(dicCamera.data)

        print(f'\n\n\n{self.dicCamera}\n\n\n')
        self.coef_angular = self.dicCamera['coef_angular']
        self.x_linha = self.dicCamera['x_linha']
        self.ids = self.dicCamera['ids']
        self.media = self.dicCamera['media']
        self.centro = self.dicCamera['centro']
        self.distancianp = self.dicCamera['distancianp']
        self.resultados = self.dicCamera['resultados']
        self.rvec = self.dicCamera['rvec']
        
    def callbackLaser(self, dicLaser):
        # conversao json (lembrar do .data)
        self.dicLaser = json.loads(dicLaser.data)

        self.laserDadoFrente = self.dicLaser["laserDadoFrente"]
        self.laserDadoSairCentro = self.dicLaser["laserDadoSairCentro"]
        self.laserDadoCreeper = self.dicLaser["laserDadoCreeper"]
    

    def callbackOdo(self, angle_z):
        self.dicOdo = json.loads(angle_z.data)
        self.angulo_local = self.dicOdo



        
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

        # self.OBJETIVONATELA = False
        # self.CREEPERNAMAO = False
        # self.tempoCreeper = True                


def main(args):

  ''' FUNCAO QUE DEFINE OS OBJETIVOS'''

  ling = Lingling("blue", 22, "car") # ========>>>>> MUDAR AQUI


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)