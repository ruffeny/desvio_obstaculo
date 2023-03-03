#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
import numpy as np
from numpy.linalg import inv



def newOdom(msg):
	global x, y, theta
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	print("x = ",x," e y = ",y)

def calculate_attractive_force(robot_pos, goal_pos):
	inc_x = goal_pos[0] - robot_pos[0]
	inc_y = goal_pos[1] - robot_pos[1]
	u = sqrt(inc_x**2 + inc_y**2)
	if u == 0:
        	return (0, 0)
	attractive_force = 50*((1/u) * inc_x, (1/u) * inc_y)
	return attractive_force

def calculate_repulsive_force(robot_pos, obstacle_pos):
	inc_x = robot_pos[0] - obstacle_pos[0]
	inc_y = robot_pos[1] - obstacle_pos[1]
	d = sqrt(inc_x**2 + inc_y**2)
	if d < obstacle_distance_threshold:
		repulsive_force = 0.5*(1/d) * (inc_x), (1/d)*(inc_y)
		if d == 0:
        		return (0, 0)
	else:
		repulsive_force = (0, 0)
	return repulsive_force

def calculate_net_force(robot_pos, goal_pos, obstacles_positions):
	net_force = calculate_attractive_force(robot_pos, goal_pos)
	for obstacle_pos in obstacles_positions:
		repulsive_force = calculate_repulsive_force(robot_pos, obstacle_pos)
		net_force = (net_force[0] + repulsive_force[0], net_force[1] + repulsive_force[1])
	return net_force

def force_to_velocities(force, robot_mass, robot_inertia, force_application_point):
    torque = np.cross(force_application_point, force)
    linear_velocity = force / robot_mass
    angular_velocity = np.dot(inv(robot_inertia), torque)
    return linear_velocity, angular_velocity
    
x = 0.0
y = 0.0
theta = 0.0


#force = np.array([0.1, 0.2, 0.3])
husky_mass = 46.03
husky_inertia = np.array([[0.06, -0.02, -0.12], [1.74, 0, 2.03],[0.01,0.01,0.1]])


rospy.init_node("speed_controller")
sub = rospy.Subscriber("/odom_husky", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(2) # 10 hz

goal = Point()	
goal.x = -5
goal.y = -5
obstacles = [(3,3), (-2,-2), (-2,-2), (-2,-2), (-2,-2)]
obstacle_radius = 0.5
obstacle_distance_threshold = obstacle_radius + 2 # Threshold distance

while not rospy.is_shutdown():
	robot_pos = (x, y)
	net_force = calculate_net_force(robot_pos, (goal.x, goal.y), obstacles)
	force = np.array(net_force)
	force_application_point = np.array(robot_pos)
	linear_velocity, angular_velocity = force_to_velocities(force, husky_mass, husky_inertia, force_application_point)
	angle = atan2(net_force[1], net_force[0])
	speed.linear.x = linear_velocity[0]	
	speed.linear.x = linear_velocity[1]
	z_axis_rotation = angular_velocity[2,2]
	speed.angular.z = z_axis_rotation/180*2*3.14
	
	print("angular_velocity = ",z_axis_rotation/360, "fim")
	pub.publish(speed)
	r.sleep()

