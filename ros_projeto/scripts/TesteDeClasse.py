#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import rospy
from std_msgs.msg import Float64, String
#from modulos_Externos import ModulosExternos

def callback(angulo):   
    print(angulo.data)
    angulo_adap = json.loads(angulo.data)
    
    print("DIC", angulo_adap)
    
rospy.init_node("teste")
try:
    contador = 0
    while not rospy.is_shutdown():
        if contador % 4000:

            rospy.Subscriber("DadosLaser", String, callback)
            contador = 0
        else:
            contador += 1
        rospy.sleep(0.1)
except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
        