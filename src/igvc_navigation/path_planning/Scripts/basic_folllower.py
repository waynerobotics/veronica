#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0

xGoal = 0.0
yGoal = 0.0

def setWaypoint(msg):
    global xGoal, yGoal
    xGoal = msg.linear.x
    yGoal = msg.linear.y

goal_sub = rospy.Subscriber("/basic_waypoint", Twist, setWaypoint)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("basic_follower")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()
goal = Point()

r = rospy.Rate(10)

goal.x = xGoal
goal.y = yGoal

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()   
 

# rostopic pub -r 10 /basic_waypoint geometry_msgs/Twist  "{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.0}}"
