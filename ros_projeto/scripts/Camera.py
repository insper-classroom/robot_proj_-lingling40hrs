#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, String
from tf import transformations
from tf import TransformerROS
import tf2_ros

import visao_module
import regressaoLinear as rl
import corModule as cM

calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

#---------------------------Parâmetros do aruco--------------------------------
#Define the aruco dictionar
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000

#Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN
scan_dist = 0

bridge = CvBridge()

#Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
#Descarta imagens que chegam atrasadas demais

check_delay = False 

frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()

#----------------------------Variáveis do Aruco----------------------------

distancenp = 0
id = 0
rvec = 0