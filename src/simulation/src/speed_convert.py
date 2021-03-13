#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32

radius=0.2
langular_speed=0
rangular_speed =0

#This node will read both the left and right wheel
# velocities and update the gazebo model (publish these to gazebo)
#call back for the left subscribe

def lconverter(data):
	langular_speed=data.data/radius
	publ.publish (langular_speed);
	print (langular_speed)

#call back for the right subcribe
def rconverter(data):
	rangular_speed=data.data/radius
	pubr.publish (rangular_speed);
	print (rangular_speed)

#initialize the node
rospy.init_node("gazebo_speed_converter")

#set the rate to be 30Hz
rate=rospy.Rate(30)

#subscribe to both left and right subscriber
rospy.Subscriber("/rwheel_vtarget",Float32 , rconverter)
rospy.Subscriber("/lwheel_vtarget",Float32 , lconverter)

#set up two gazebo publishers for left and right 
publ=rospy.Publisher("/wayne_robotics_model/joint_left_velocity/command", Float64, queue_size=10)
pubr=rospy.Publisher("/wayne_robotics_model/joint_right_velocity/command", Float64, queue_size=10)


#msg=Float64()

while not rospy.is_shutdown():
	#msg.data=angular_speed
	#pub.publish(msg)
	#print(angular_speed)
	rate.sleep()
