#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError
import cormodule
import visao_module
from imutils.video import VideoStream
import argparse
import imutils

bridge = CvBridge()
procurando = True
achado = False
cv_image = None
frame = None
media = []
centro = []
atraso = 0.3E9 # 1 segundo e meio. Em nanossegundos
centro_img = (640/2,480/2)
area = 0.0 # Variavel com a area do maior contorno
check_delay = False

lidar = []
tracking = False
counter = 0
start_track = False
track_center = ()
result_tuples = []
bumper = 0
v = 0.5
w = 0.8
def roda_todo_frame(imagem):
	global counter
	global start_track
	global cv_image
	global media
	global centro
	global centro_img
	global result_tuples
	global frame
	global tracking
	global tracker
	global track_center
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area =  cormodule.identifica_cor(cv_image)
		result_frame, result_tuples = visao_module.mnet.detect(cv_image)
		frame = result_frame
		depois = time.clock()

		if len(result_tuples)>0:
			counter+=1
			if counter == 5:
				start_track = True
				tracking = True
		if start_track:
			startX, startY = result_tuples[0][2]
			endX, endY = result_tuples[0][3]
			l = endX - startX
			h = endY - startY
			initBB = (startX,startY,l,h)


			OPENCV_OBJECT_TRACKERS = {
				"csrt": cv2.TrackerCSRT_create,
				"kcf": cv2.TrackerKCF_create,
				"boosting": cv2.TrackerBoosting_create,
				"mil": cv2.TrackerMIL_create,
				"tld": cv2.TrackerTLD_create,
				"medianflow": cv2.TrackerMedianFlow_create,
				"mosse": cv2.TrackerMOSSE_create
			}
			tracker = OPENCV_OBJECT_TRACKERS["kcf"]()

			(H, W) = frame.shape[:2]

		if start_track:
			tracker.init(result_frame, initBB)
			start_track = False

		if tracking:
			(success, box) = tracker.update(result_frame)
			if success:
				(x, y, w, h) = [int(v) for v in box]
				cv2.rectangle(result_frame, (x, y), (x + w, y + h),
					(0, 255, 0), 2)
				track_center = ((x+(w/2)),(y+(h/2)))
				cv2.circle(result_frame,track_center, 5, (0,0,255), -1)
			else:
				counter = 0
				tracking = False


		key = cv2.waitKey(1) & 0xFF



		cv2.imshow("Camera", result_frame)
	except CvBridgeError as e:
		print('ex', e)

def bump(dado):
	global bumper
	bumper = dado.data

def scaneou(dado):
	global lidar
	lidar = np.array(dado.ranges).round(decimals=3)

if __name__=="__main__":
	rospy.init_node("cor")
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, bump)
	topico_imagem = "/kamera"
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3)
	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

	try:
		while not rospy.is_shutdown():
			#--------- Bumper ---------#

			if bumper == 4:
				print("bumper 1",bumper)
				vel = Twist(Vector3(-v,0,0), Vector3(0,0,w))
				velocidade_saida.publish(vel)
				rospy.sleep(2)
				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
			elif bumper == 3:
				print("bumper 2",bumper)
				vel = Twist(Vector3(-v,0,0), Vector3(0,0,-w))
				velocidade_saida.publish(vel)
				rospy.sleep(2)
				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
			elif bumper == 2:
				print("bumper 3",bumper)
				vel = Twist(Vector3(2*v,0,0), Vector3(0,0,w))
				velocidade_saida.publish(vel)
				rospy.sleep(2)
				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
			elif bumper == 1:
				print("bumper 4",bumper)
				vel = Twist(Vector3(2*v,0,0), Vector3(0,0,-w))
				velocidade_saida.publish(vel)
				rospy.sleep(2)
				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
			bumper = 0
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

			#----------LASER------------#

			for i in range(len(lidar)):
				if lidar[i]>0 and lidar[i]<0.15:
					if i >= 45 and i < 135:
						print("Objeto detectado no angulo",i)
						vel = Twist(Vector3(v,0,0), Vector3(0,0,-w))
						velocidade_saida.publish(vel)
						rospy.sleep(1)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(0.01)

					if i >= 135 and i < 225:
						print("Objeto detectado no angulo",i)
						vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(0.01)

					if i >= 225 and i <= 315:
						print("Objeto detectado no angulo",i)
						vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
						velocidade_saida.publish(vel)
						rospy.sleep(1)
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(0.01)

			#-----------TRACKING----------#

			tresh = 30

			if tracking:
				if len(track_center)>0 :
					if len(lidar) != 0 and lidar[0] != 0 :
						dist = lidar[0]
						vt = dist*0.2
						#print(vt)
					else:
						vt = 0
					if  track_center[0] < centro_img[0]-tresh or track_center[0] > centro_img[0]+tresh:
						print("fora do centro ")
						wt = ((centro_img[0]-track_center[0])*0.001)
						#print(wt)
					else:
						wt=0
					print(vt)
					vel = Twist(Vector3(vt,0,0), Vector3(0,0,wt))
					velocidade_saida.publish(vel)
					rospy.sleep(0.05)


			#-------- REAGE COR - VERDE -------------#
			if len(media) > 0:
				if media[0] != 0:
					print(len(media))
					print(media)
					print(centro)
					vel = Twist(Vector3(0,0,0), Vector3(0,0,1))
					velocidade_saida.publish(vel)
					rospy.sleep(7)


	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
