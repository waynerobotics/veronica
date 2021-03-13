#!/usr/bin/env python

#simple simulator for use with rviz, publish the left and right wheel angular velocities on:
#/left_speed_setpoint
#/right_speed_setpoint

import rospy
import math
import tf

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

#Note: These speeds can never be exactly the same or simulation will break
left_speed=0.00001
right_speed=0.00002
robot_x=0 #Holds robots current x position
robot_y=0 #Holds robots current y position
robot_z=0.30795 #Constant to bring the robot wheels to the ground
robot_heading=0 #Holds robots current heading/direction

time_step=0.01 #Time between system updates (directly affects quality of rviz movement)
l=0.889 #Width between the centers of the wheels
w_r=0.2 #Radius of wheel

def left_speed_callback(data):
	global left_speed
	left_speed=data.data
	if left_speed == 0: #ensures the formulas do not break. Shouldnt affect the output much if it is sitting still
		left_speed=0.000000000001


def right_speed_callback(data):
	global right_speed
	right_speed=data.data
	if right_speed == 0: #ensures the formulas do not break. Shouldnt affect the output much if it is sitting still
		right_speed=0.000000000002

rospy.init_node("RVIZ Transform Broadcaster")
rospy.Subscriber('left_speed_setpoint', Float64, left_speed_callback)
rospy.Subscriber('right_speed_setpoint', Float64, right_speed_callback)

br=tf.TransformBroadcaster()#allows rviz to plot the robot position

rate=rospy.Rate(1/time_step)

while not rospy.is_shutdown():

	#Broadcast the frame transformations for each joint in the robot plus one between the base link and the map
	br.sendTransform((robot_x,robot_y,robot_z),
		quaternion_from_euler(0,0,robot_heading),
		rospy.Time.now(),
		"base_link",
		"map")
	br.sendTransform((0,0.4445,-0.10795),
		quaternion_from_euler(0,0,0),
		rospy.Time.now(),
		"left_wheel",
		"base_link")
	br.sendTransform((0,-0.4445,-0.10795),
		quaternion_from_euler(0,0,0),
		rospy.Time.now(),
		"right_wheel",
		"base_link")
	br.sendTransform((-0.6,0,-0.18295),
		quaternion_from_euler(0,0,0),
		rospy.Time.now(),
		"rear_stand",
		"base_link")
	br.sendTransform((-0.05,0,-0.075),
		quaternion_from_euler(0,0,0),
		rospy.Time.now(),
		"rear_stand_wheel",
		"rear_stand")

	#Update the position of the robot
	R=(l/2)*(left_speed+right_speed)/(right_speed-left_speed)
	w=(right_speed-left_speed)/l

	ICCx=robot_x-R*math.sin(robot_heading)
	ICCy=robot_y+R*math.cos(robot_heading)

	robot_x=math.cos(w*time_step)*R*math.sin(robot_heading)-math.sin(w*time_step)*-R*math.cos(robot_heading)+ICCx
	robot_y=math.sin(w*time_step)*R*math.sin(robot_heading)+math.cos(w*time_step)*-R*math.cos(robot_heading)+ICCy
	robot_heading=robot_heading+w*time_step

	rate.sleep()




