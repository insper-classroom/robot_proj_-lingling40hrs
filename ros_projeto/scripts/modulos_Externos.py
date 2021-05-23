#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy 
import math
import numpy as np
from tf import transformations
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class ModulosExternos:
    def __init__(self):
        self.pubLaserDadoFrente = rospy.Publisher("laserFrente",Float64,queue_size=1) 
        self.pubLaserDadoSairCentro = rospy.Publisher("laserSairCentro",Float64,queue_size=1)
        self.pubLaserDadoCreeper = rospy.Publisher("laserCreeper",Float64,queue_size=1) 
        
        self.pubAngulo = rospy.Publisher("Angulo",Float64,queue_size=1)

        self.laserDadoFrente = 10
        self.laserDadoSairCentro = 10
        self.laserDadoCreeper = 10

        self.angle_deg = 0
        
        rospy.init_node("ModulosExternos")
        def scaneou(dado):
            self.laserDadoFrente = dado.ranges[0]
            self.laserDadoSairCentro = dado.ranges[90]
            laser0to15 = np.min(dado.ranges[0:45])
            laser345to360 = np.min(dado.ranges[315:360])
            listaMinimosLaser = [laser0to15, laser345to360]
            self.laserDadoCreeper = np.min(listaMinimosLaser)
        self.pubLaserDadoFrente.publish(self.laserDadoFrente)
        self.pubLaserDadoSairCentro.publish(self.laserDadoSairCentro)
        self.pubLaserDadoCreeper.publish(self.laserDadoCreeper)

        

        def recebe_odometria(data):
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y

            quat = data.pose.pose.orientation
            lista = [quat.x, quat.y, quat.z, quat.w]
            radianos = transformations.euler_from_quaternion(lista)
            angle_z = radianos[2]   
            self.angle_deg = math.degrees(angle_z)
            if self.angle_deg < 0:
                self.angle_deg = self.angle_deg + 360.00
        self.pubAngulo.publish(self.angle_deg)
        
        
        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
        self.ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
        
try:
    mod = ModulosExternos()
    while not rospy.is_shutdown():
        mod.pubAngulo.publish(mod.angle_deg)
        mod.pubLaserDadoFrente.publish(mod.laserDadoFrente)
        mod.pubLaserDadoCreeper.publish(mod.laserDadoCreeper)
        mod.pubLaserDadoSairCentro.publish(mod.laserDadoSairCentro)
except rospy.ROSInterruptException:
    print("Ocorreu uma exceção com o rospy")