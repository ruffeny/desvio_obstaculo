#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from numpy.linalg import inv
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelState
import obstaculo as obst

def defineObstModel(model,name,pos,vel):
	model.model_name = name
	model.pose.position.x = pos[0]
	model.pose.position.y = pos[1]
	model.twist.linear.x = vel[0]
	model.twist.linear.y = vel[1]
	
	return model
	
def getOdom(msg):
	global x, y, theta, v_husky, w_husky
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y	
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
	v_husky.x = msg.twist.twist.linear.x
	v_husky.y = msg.twist.twist.linear.y
	w_husky.z = msg.twist.twist.angular.z	
	#print("x = ",x," e y = ",y, "velocity = ",v_gazebo,"angular velocity = ",w_gazebo)
	
def getOdomBox(msg):
	global x_box, y_box, theta_box, v_gazebo_box, w_gazebo_box
	x_box = msg.pose.pose.position.x
	y_box = msg.pose.pose.position.y	
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta_box) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
	v_gazebo_box.x = msg.twist.twist.linear.x
	v_gazebo_box.y = msg.twist.twist.linear.y
	w_gazebo_box.z = msg.twist.twist.angular.z	
	#print("x = ",x," e y = ",y, "velocity = ",v_gazebo,"angular velocity = ",w_gazebo)
	
def setParam():
	#APF  Modificado - Parâmetros
	epsilon = 30000;#3000
	Nd = 20000;#2000 velocidade diferente de zero
	Ns = 3000000;#300000  velocidade isgual de zero
	Ne = 20000;#2000  emergencia
	tau = 0.3; #mn milha nautica = 1852 metros
	Ros = 0.8; #mn
	Dsafe = 1; #0.5mn ou 1 se for mar aberto
	phou0 = 3; #3 ate 5mn
	maxturn = 90 ; #5 graus
	timeStep = 15; #segundos
	Rts = 0.5; #mn
	vmax = 0.4; #m/s
	#wmax = math.radians(maxturn);
	wmax = vmax/Ros
	Dm = Ros + Dsafe + Rts;
	CR = Dm + phou0;
	
	return epsilon, Nd, Ns, Ne, tau, Ros, Dsafe, phou0, maxturn, Rts, vmax, wmax, Dm, CR

def paramInicial():
	x = 0.0
	y = 0.0
	v_husky = Vector3()
	w_husky = Vector3()
	theta = 0.0
	x_box = 0.0
	y_box = 0.0
	v_gazebo_box = Vector3()
	w_gazebo_box = Vector3()
	theta_box = 0.0
	
	return x,y,v_husky,w_husky, theta, x_box,y_box,v_gazebo_box,w_gazebo_box,theta_box


# inicializacao dos parametros
epsilon, Nd, Ns, Ne, tau, Ros, Dsafe, phou0, maxturn, Rts, vmax, wmax, Dm, CR = setParam()

x,y,v_husky,w_husky, theta, x_box,y_box,v_gazebo_box,w_gazebo_box,theta_box = paramInicial()

print(Ros)
#inicio do node
rospy.init_node("apf_controller")
sub = rospy.Subscriber("/odom_husky", Odometry, getOdom) #pega informação da posicao e velocidade do husky
pub = rospy.Publisher("/twist_marker_server/cmd_vel", Twist, queue_size=1) #aplica no topico do husky
speed = Twist()

sub_box = rospy.Subscriber("/odom_box_1", Odometry, getOdomBox) #pega incormação odom do obstaculo
pub_box = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1) #algera informações do obstaculo
model_state = ModelState()

#defineObstModel(name,pos,vel)
pos_obst = np.array([8,4])
vel = np.array([-0.1,0.1])
#model_state = defineObstModel(model_state,'custom_ground_plane_box',pos_obst,np.array([0,0]))
obst.change_position_velocity('custom_box_1',pos_obst,vel)
speed_box = Twist()


#gerar obstaculos pos,radius,vel
obstacles = [np.array([model_state.pose.position.x, model_state.pose.position.y])]
obstacles_radius = [Rts]
obstacles_velocities = [np.array([model_state.twist.linear.x, model_state.twist.linear.y])]

#defini posição goal
goal_pos = np.array([10, 10])


name='husky'
obst.change_position(name,np.array([0,0]))



r = rospy.Rate(1000) # 5 hz
i = 1
k = 1
robot_positions = []
while not rospy.is_shutdown():
	if i <= 5:
		i = i + 1
	# get robot position
	robot_pos = np.array([x, y])
	obst.save_robot_position(robot_positions,x, y)
	
	#own ship velocity
	vos_velocity = np.array([v_husky.x,v_husky.y])
	
	obstacles = [np.array([x_box, y_box])]
	obstacles_radius = [Rts]
	obstacles_velocities = [np.array([v_gazebo_box.x , v_gazebo_box.y])]

	total_force = obst.modified_potential_field(goal_pos,obstacles,obstacles_radius,obstacles_velocities,robot_pos,epsilon,Nd,Ns,Ne,tau,Ros,Dsafe,phou0,vos_velocity)
	
	#print("F_total = ",total_force)
	v,w = obst.force_to_velocities(total_force, theta)
	speed.linear.x = min(v, vmax)
	speed.angular.z = np.sign(w) * min(abs(w), wmax)
	#print("posicao atual = ",robot_pos,"obstacles = ",obstacles,"total_force = ",total_force)
	distance_to_goal = np.linalg.norm(goal_pos - robot_pos) 
	#print("distancia = ",distance_to_goal," v = ",speed.linear.x," w =",speed.angular.z)

	if distance_to_goal < 0.5:
		speed.linear.x = 0
		speed.angular.z = 0	
		model_state.twist.linear.x = 0
		model_state.twist.linear.y = 0
		pub_box.publish(model_state)
		if k == 1:
			obst.plot_robot_positions(robot_positions,pos_obst,Dm)
			k = K + 1
		break
	if i>5:
		pub.publish(speed)
	r.sleep()

