#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64
#from modulos_Externos import ModulosExternos

def callback(angulo):
    print("MEU ANGULO È:", angulo)

rospy.init_node("teste")
try:
    contador = 0
    while not rospy.is_shutdown():
        if contador % 4000:

            rospy.Subscriber("Angulo", Float64, callback)
            contador = 0
        else:
            contador += 1
        rospy.sleep(0.1)
except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")