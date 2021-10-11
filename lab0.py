#! /usr/bin/python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import math

class MoveTurtle:
	def __init__(self):
		rospy.Subscriber('/turtle1/pose', Pose, self.callback1)
		rospy.Subscriber('/leo/pose', Pose, self.callback2)
		self.my_y = 0
		self.my_x = 0
		self.theta = 0
		self.pub_leo = rospy.Publisher('/leo/cmd_vel', Twist, queue_size = 1)
        
	def callback1(self, msg):
		rospy.logerr(msg)
		msg_new = Twist()
		if self.is_next(msg):
			return
		msg_new.linear.x = self.speed_turtle(msg)
        msg_new.angular.z = self.angle(msg)         
		self.pub_leo.publish(msg_new)
        
	def callback2(self, msg):
		self.my_x = msg.x
		self.my_y = msg.y
		self.theta = msg.theta
        
	def is_next(self, msg):
		return abs(self.my_x - msg.x) < 0.1 and abs(self.my_y - msg.y) < 0.1
        
    def speed_turtle(self, msg):
		return math.sqrt((msg.x - self.my_x) ** 2 + (msg.y - self.my_y) ** 2) / 10
        
    def angle(self, msg):
        return 1.5 * (math.atan2(msg.y - self.my_y, msg.x - self.my_x) - self.theta)    

rospy.init_node('turtle_init')
MoveTurtle()
rospy.spin()