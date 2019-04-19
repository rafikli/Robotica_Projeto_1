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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule


bridge = CvBridge()
procurando = True
achado = False
cv_image = None
media = []
centro = []
atraso = 0.3E9 # 1 segundo e meio. Em nanossegundos
centro_img = (640/2,480/2)
area = 0.0 # Variavel com a area do maior contorno
# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global centro_img
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area =  cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/kamera"
	
	# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				tresh = 30
				print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))
				if procurando:
					if  centro[1]-tresh > media[0]:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
						velocidade_saida.publish(vel)
					elif centro[1]+tresh < media[0]:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
						velocidade_saida.publish(vel)
					elif media[0] < centro[1]+tresh and media[0] > centro[1]-tresh:
						procurando = False
						achado = True
				if achado:
					if centro[1]-tresh > media[0] or centro[1]+tresh < media[0]:
						achado = False
						procurando =True
					else:
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel) 
			rospy.sleep(0.01)		
		

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


