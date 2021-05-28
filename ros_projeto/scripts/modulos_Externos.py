#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy 
import math
import sys
import json
import numpy as np
from tf import transformations
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class ModulosExternos:
    def __init__(self):        
        self.pubAngulo = rospy.Publisher("Angulo",String,queue_size=1)

        rospy.init_node("ModulosExternos", anonymous=True)

        self.laserDadoFrente = 10
        self.laserDadoSairCentro = 10
        self.laserDadoCreeper = 10

        self.angle_deg = 0

        self.ref_odometria = rospy.Subscriber("/odom", Odometry, self.recebe_odometria)
        
    def recebe_odometria(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        radianos = transformations.euler_from_quaternion(lista)
        angle_z = radianos[2]   
        self.angle_deg = math.degrees(angle_z)
        if self.angle_deg < 0:
            self.angle_deg = self.angle_deg + 360.00
        self.angle_deg = json.dumps(self.angle_deg)
        self.pubAngulo.publish(self.angle_deg)
        
def main(args):
  md = ModulosExternos()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)