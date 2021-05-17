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


class garra:
    def __init__(self):
        self.arm_publisher = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=1)
        self.garra_publisher = rospy.Publisher('/joint2_position_controller/command',Float64,queue_size=1)

        self.arm_state = -1
        self.garra_state = 0
        self.time = 0.1

    def inicializar_garra(self):
        self.arm_state = -1
        self.garra_state =  0
        
        self.arm_publisher.publish(self.arm_state) #abaixa 
        rospy.sleep(self.time)

        self.garra_publisher.publish(self.garra_state) #fecha 
        rospy.sleep(self.time)

    def open(self):
        self.garra_state = -1
        self.garra_publisher.publish(self.garra_state)
        rospy.sleep(self.time)

    def close(self):
        self.garra_state = 0
        self.garra_publisher.publish(self.garra_state)
        rospy.sleep(self.time)
        
    def up(self):
        if self.arm_state == -1:
            self.arm_state = 0
        elif self.arm_state == 0:
            self.arm_state = 1.5
        
        self.arm_publisher.publish(self.arm_state)
        rospy.sleep(self.time)

    def down(self):
        if self.arm_state == 1.5:
            self.arm_state = 0
        elif self.arm_state == 0:
            self.arm_state = -1
        
        self.arm_publisher.publish(self.arm_state)
        rospy.sleep(self.time)