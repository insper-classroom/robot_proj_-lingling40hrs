import rospy
import string
import math
import LingLingRobot as Lingling

class Movimento:
    def __init__(self, giroCompleto):
        self.v = None
        self.w = None

        def andar(coef_angular, x_linha):
            meio_linha = (x_linha[0] + x_linha[1])/2
            erro_x = meio_linha - 360
            v = 0.4- abs(erro_x)/900 - 0.16/laserDadoFrente 
            erro_coefang = coef_angular
            w = (-erro_x/800) + (erro_coefang/160) 

            return self.v,self.w

        def giro90 (angulo_local, angulo_fin):
            giroCompleto = False
            angulo_init = angulo_fin + 90
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
                        w = 0
                        giroCompleto = True
                        return self.v, self.w, giroCompleto
                    self.w = 0.1
            return self.v, -self.w, giroCompleto

        def giro180 (angulo_local, angulo_fin):
            giroCompleto = False
            dif = abs(angulo_local- angulo_fin)
            self.v = 0
            self.w = 0
            if dif <= 5:
                giroCompleto = True
                return self.v, self.w, giroCompleto
            self.w = -0.2
            
            return self.v, self.w, giroCompleto