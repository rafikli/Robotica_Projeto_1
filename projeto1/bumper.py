#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8

v = 0.5  # Velocidade linear
w = 0.8  # Velocidade angular
bumper = 0
def bump(dado):	
	global bumper
	bumper = dado.data


if __name__ == "__main__":
    rospy.init_node("roda_exemplo")

    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
    recebe_bumper = rospy.Subscriber("/bumper", UInt8, bump)
    vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
    pub.publish(vel)
    try:
        while not rospy.is_shutdown():
            if bumper == 1:
            	print("bumper 1",bumper)
            	vel = Twist(Vector3(-v,0,0), Vector3(0,0,w))
            	pub.publish(vel)
                rospy.sleep(2)
                vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                pub.publish(vel)
            elif bumper == 2:
            	print("bumper 2",bumper)
            	vel = Twist(Vector3(-v,0,0), Vector3(0,0,-w))
            	pub.publish(vel)
                rospy.sleep(2)
                vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                pub.publish(vel)
            elif bumper == 3:
            	print("bumper 3",bumper)
            	vel = Twist(Vector3(2*v,0,0), Vector3(0,0,w))
            	pub.publish(vel)
                rospy.sleep(2)
                vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                pub.publish(vel)
            elif bumper == 4:
            	print("bumper 4",bumper)
            	vel = Twist(Vector3(2*v,0,0), Vector3(0,0,-w))
            	pub.publish(vel)
                rospy.sleep(2)
                vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                pub.publish(vel)
            bumper = 0
            


    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")

	


