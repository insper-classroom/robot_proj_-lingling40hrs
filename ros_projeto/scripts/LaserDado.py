#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy 
import json
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LaserDado:
    def __init__(self):
        self.pubDadosLaser = rospy.Publisher("DadosLaser", String, queue_size=1)

        rospy.init_node("LaserDado", anonymous=True)

        self.laserDadoFrente = 10
        self.laserDadoSairCentro = 10
        self.laserDadoCreeper = 10

        self.dicionarioDados = {}
        self.dictJson = ""

        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)

    def scaneou(self, dado):
        self.laserDadoFrente = dado.ranges[0]
        self.laserDadoSairCentro = dado.ranges[90]
        laser0to15 = np.min(dado.ranges[0:45])
        laser345to360 = np.min(dado.ranges[315:360])
        listaMinimosLaser = [laser0to15, laser345to360]
        self.laserDadoCreeper = np.min(listaMinimosLaser) 
        self.dicionarioDados["laserDadoFrente"] = self.laserDadoFrente
        self.dicionarioDados["laserDadoSairCentro"] = self.laserDadoSairCentro
        self.dicionarioDados["laserDadoCreeper"] = self.laserDadoCreeper
        self.dictJson = json.dumps(self.dicionarioDados)
        print(self.dictJson,type(self.dictJson))
        print("DICT", self.dicionarioDados)
        self.hello = "hello, I am here"
        self.pubDadosLaser.publish(self.dictJson)

def main(args):
  ld = LaserDado()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)