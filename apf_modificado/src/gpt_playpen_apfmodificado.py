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
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from matplotlib.patches import Circle
from scenarios import predefined_scenarios



def plot_obstacles(obstacles):
    global x,y
    global total_force
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)

    def update(frame):
        ax.clear()
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        for i, obstacle in enumerate(obstacles):
            ax.scatter(obstacle.x, obstacle.y, marker='x', color='red')
            obstacle_circle = Circle((obstacle.x, obstacle.y), obstacles_radius[i], fill=False, edgecolor='red', linestyle='--')
            ax.add_artist(obstacle_circle)
        ax.scatter(x, y, marker='o', color='blue')
        husky_circle_ros = Circle((x, y), Ros, fill=False, edgecolor='blue', linestyle='--')
        husky_circle_cr = Circle((x, y), CR, fill=False, edgecolor='green', linestyle='-.')
        husky_circle_dm = Circle((x, y), Dm, fill=False, edgecolor='purple', linestyle=':')
        ax.add_artist(husky_circle_ros)
        ax.add_artist(husky_circle_cr)
        ax.add_artist(husky_circle_dm)
        force_norm = total_force / np.linalg.norm(total_force)
        ax.quiver(x, y, force_norm[0], force_norm[1], color='black', angles='xy', scale_units='xy', scale=1)


    ani = FuncAnimation(fig, update, frames=range(0, 1000), interval=100, blit=False)
    plt.show()



class Obstacle:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v_gazebo = 0
        self.w_gazebo = 0
        
    def get_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y	
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.v_gazebo = msg.twist.twist.linear.x
        self.w_gazebo = msg.twist.twist.angular.z
        
        
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
	epsilon = 6000;#3000
	Nd = 20000;#2000 velocidade diferente de zero
	Ns = 300000;#300000  velocidade igual de zero
	Ne = 4000;#2000  emergencia
	tau = 0.3; # 0.3 milhas náuticas são equivalentes a 555,6 metros
	Ros = 0.5; #mn
	Dsafe = 1; #0.5mn ou 1 se for mar aberto
	phou0 = 5; #3 ate 5mn
	maxturn = 45 ; #5 graus
	timeStep = 15; #milisegundos
	Rts = 1; #mn
	vmax = 0.4; #m/s
	#wmax = math.radians(maxturn);
	wmax = math.radians(maxturn)#vmax/Ros
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

def plot_simulation_positions(robot_positions, obstacle_positions, sim_time, time_step=0.1):
    fig, ax = plt.subplots()
    ax.set_xlim(-1, 10)
    ax.set_ylim(-1, 10)

    num_steps = int(sim_time / time_step)
    
    # Desenhar círculos de obstáculos fixos na posição inicial
    initial_obstacles = obstacle_positions[0]
    for j, obstacle in enumerate(initial_obstacles):
        obstacle_circle = Circle((obstacle[0], obstacle[1]), obstacles_radius[j], fill=False, edgecolor='red', linestyle='--')
        ax.add_artist(obstacle_circle)

    # Desenhar círculo azul na posição inicial para representar o Husky
    initial_robot_position = robot_positions[0]
    husky_circle = Circle((initial_robot_position[0], initial_robot_position[1]), 0.5, fill=False, edgecolor='blue', linestyle='solid')
    ax.add_artist(husky_circle)
    
    # Desenhar o traço do centro dos círculos representando o movimento do obstáculo
    for obstacles in obstacle_positions[:num_steps]:
        for j, obstacle in enumerate(obstacles):
            ax.scatter(obstacle[0], obstacle[1], marker='x', color='red')
    
    robot_x = [pos[0] for pos in robot_positions[:num_steps]]
    robot_y = [pos[1] for pos in robot_positions[:num_steps]]
    ax.plot(robot_x, robot_y, 'bo-', markersize=2)
    
    plt.title(f"Simulação de {sim_time} segundos")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.show()

def plot_distances(distances_list, obstacles_radius,Ros, time_step=0.1):
    plt.figure()
    for i, distances in enumerate(distances_list):
        plt.plot(distances, label=f'Obstáculo {i+1}')
    plt.axhline(y=obstacles_radius[0]+Ros, color='r', linestyle='--', label='Raio do obstáculo')
    plt.title('Tempo vs distância até o obstáculo')
    plt.xlabel('Tempo (x.' + str(time_step) + ' s)')
    plt.ylabel('Distância (m)')
    #plt.legend()
    plt.show()

def plot_linear_velocities(linear_velocities, linear_velocities_control, time_step=0.1):
    plt.figure()
    time = np.arange(0, len(linear_velocities) * time_step, time_step)
    plt.plot(time, linear_velocities, label='Velocidade linear')
    plt.plot(time, linear_velocities_control, label='sinal de controle')
    plt.title('Evolução das velocidades lineares no tempo')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Velocidade (m/s)')
    plt.legend()
    plt.show()

def plot_angular_velocities(angular_velocities, angular_velocities_control, time_step=0.1):
    plt.figure()
    time = np.arange(0, len(angular_velocities) * time_step, time_step)
    plt.plot(time, angular_velocities, label='Velocidade angular')
    plt.plot(time, angular_velocities_control, label='sinal de controle')
    plt.title('Evolução das velocidades angulares no tempo')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Velocidade (rad/s)')
    plt.legend()
    plt.show()
    
try:
	# inicializacao dos parametros
	epsilon, Nd, Ns, Ne, tau, Ros, Dsafe, phou0, maxturn, Rts, vmax, wmax, Dm, CR = setParam()

	x,y,v_husky,w_husky, theta, x_box,y_box,v_gazebo_box,w_gazebo_box,theta_box = paramInicial()

	list_robot_positions = []
	list_obstacle_positions = []
	distances_list = []
	linear_velocities = []
	linear_velocities_control = []
	angular_velocities = []
	angular_velocities_control = []

	selected_scenario = predefined_scenarios('scenario_5')
	goal_pos = selected_scenario['goal_pos']
	pos_obst = selected_scenario['obstacle_positions']
	obstacle_velocities = selected_scenario['obstacle_velocities']
	obstacles_radius = selected_scenario['obstacle_radii']

	rospy.set_param('use_sim_time', True)

	print("inicio do codigo")
	#inicio do node
	rospy.init_node("apf_controller")
	sub = rospy.Subscriber("/odom_husky", Odometry, getOdom) #pega informação da posicao e velocidade do husky
	pub = rospy.Publisher("/twist_marker_server/cmd_vel", Twist, queue_size=1) #aplica no topico do husky
	speed = Twist()

	#sub_box = rospy.Subscriber("/odom_box_1", Odometry, getOdomBox) #pega incormação odom do obstaculo
	#pub_box = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1) #altera informações do obstaculo
	#model_state = ModelState()

	# Create instances of the obstacle class
	obst1 = Obstacle()
	obst2 = Obstacle()
	obst3 = Obstacle()
	obst4 = Obstacle()
	obst5 = Obstacle()
	obst6 = Obstacle()

	rospy.Subscriber('odom_obst1', Odometry, obst1.get_odom)
	rospy.Subscriber('odom_obst2', Odometry, obst2.get_odom)
	rospy.Subscriber('odom_obst3', Odometry, obst3.get_odom)
	rospy.Subscriber('odom_obst4', Odometry, obst4.get_odom)
	rospy.Subscriber('odom_obst5', Odometry, obst5.get_odom)
	rospy.Subscriber('odom_obst6', Odometry, obst6.get_odom)

	# Criação da lista de obstáculos
	obstacle_list = [obst1, obst2, obst3, obst4, obst5, obst6]

	# Definição da função plot_thread
	#def plot_thread():
	#    plot_obstacles(obstacle_list)

	# Criação e inicialização da thread de plotagem
	#plotting_thread = threading.Thread(target=plot_thread)
	#plotting_thread.start()

	#sub_obst1 = rospy.Subscriber("/odom_obst1",Odometry,getOdomObst1)
	#pub_obst1 = rospy.Published('/gazebo/set_model_state',ModelState,queue_size=1)
	#model_obst1 = Twist()

	#defineObstModel(name,pos,vel)

	#pos_obst = [np.array([7.8,2.2]),np.array([6.8,4.9]),np.array([7,0]),np.array([7.5,7]),np.array([6,8]),np.array([4,8])]
	#pos_obst = [np.array([4,4]),np.array([-3,-9]),np.array([-6,-6]),np.array([-9,-3]),np.array([-8,-8]),np.array([-9,-9])]
	#vel = [np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([-0,-0]),np.array([0,-0])] #0,514444 m/s = 1 no



	#vel = [np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0])] #0,514444 m/s = 1 no
	#model_state = defineObstModel(model_state,'custom_ground_plane_box',pos_obst,np.array([0,0]))
	obst.change_position_velocity('custom_box_1',np.array([-8,0]),np.array([0,0]))
	obst.change_position_velocity('Construction Barrel',pos_obst[0],obstacle_velocities[0]) #obst6 [5]
	obst.change_position_velocity('Construction Barrel_0',pos_obst[1],obstacle_velocities[1]) 
	obst.change_position_velocity('Construction Barrel_1',pos_obst[2],obstacle_velocities[2]) #obst3
	obst.change_position_velocity('Construction Cone',pos_obst[3],obstacle_velocities[3])
	obst.change_position_velocity('Construction Cone_0',pos_obst[4],obstacle_velocities[4]) #obst5
	obst.change_position_velocity('Construction Cone_1',pos_obst[5],obstacle_velocities[5])
	speed_box = Twist()


	#defini posição goal
	#goal_pos = np.array([9, 9])
	obstacles = [np.array([obst1.x,obst1.y]),np.array([obst2.x,obst2.y]),np.array([obst3.x,obst3.y]),np.array([obst4.x,obst4.y]),np.array([obst5.x,obst5.y]),np.array([obst6.x,obst6.y])]
	#obstacles_radius = [0.4 , 0.4 , 0.4 , 0.25 , 0.25 , 0.25]
	#obstacles_velocities = vel

	name='husky'
	obst.change_position(name,np.array([0,0]))



	r = rospy.Rate(10) # 10 hz
	i = 1
	k = 1
	robot_positions = []

	collision_detected = False

	while not rospy.is_shutdown():
		if i <= 5:
			i = i + 1
			
		# get robot position
		robot_pos = np.array([x, y])
		obst.save_robot_position(robot_positions,x, y)
		
		#own ship velocity
		vos_velocity = np.array([v_husky.x,v_husky.y])
		
				
			
		# Calcula a distância entre o Husky e cada obstáculo e armazena na lista distances_list
		for obs in range(len(obstacles)):
			dist = sqrt((x - obstacles[obs][0])**2 + (y - obstacles[obs][1])**2)
			if obstacles[obs][0] >= 0 and obstacles[obs][1] >= 0:  # Adiciona apenas as distâncias dos obstáculos no primeiro quadrante
				if len(distances_list) < len(obstacles):
					distances_list.append([dist])
				else:
					distances_list[obs].append(dist)
		
	    
	    	
			
		linear_velocities.append(v_husky.x)
		angular_velocities.append(w_husky.z)
		obstacles = [np.array([obst1.x,obst1.y]),np.array([obst2.x,obst2.y]),np.array([obst3.x,obst3.y]),np.array([obst4.x,obst4.y]),np.array([obst5.x,obst5.y]),np.array([obst6.x,obst6.y])]
		#obstacles_radius = [0.4 , 0.4 , 0.4 , 0.25 , 0.25 , 0.25]
		#obstacles_velocities = vel
		list_robot_positions.append(robot_pos)
		list_obstacle_positions.append(obstacles)

		total_force = obst.modified_potential_field(goal_pos,obstacles,obstacles_radius,obstacle_velocities,robot_pos,epsilon,Nd,Ns,Ne,tau,Ros,Dsafe,phou0,vos_velocity)
		
		#print("F_total = ",total_force)
		v,w = obst.force_to_velocities(total_force, theta)
		speed.linear.x = min(v, vmax)
		speed.angular.z = np.sign(w) * obst.adjust_angle(min(abs(w), wmax))
		linear_velocities_control.append(speed.linear.x)
		angular_velocities_control.append(speed.angular.z)
		#print("posicao atual = ",robot_pos,"obstacles = ",obstacles,"total_force = ",total_force)
		distance_to_goal = np.linalg.norm(goal_pos - robot_pos) 
		#print("distancia = ",distance_to_goal," v = ",speed.linear.x," w =",speed.angular.z)

		if distance_to_goal < 1:
			speed.linear.x = 0
			speed.angular.z = 0	
			#obst.change_position_velocity('custom_box_1',np.array([-5,-5]),np.array([0,0]))
			if k == 1:
				#obst.plot_robot_positions(robot_positions,pos_obst,Dm)
				#k = K + 1
				#obst.change_position('husky',np.array([8,8]))
				print("nada")
				break
		if i>5:
			pub.publish(speed)
			#print("obst1 = ",obst1.x, obst1.y, obst1.theta, obst1.v_gazebo, obst1.w_gazebo)              
		r.sleep()

	plot_distances(distances_list, obstacles_radius,Ros)

	plot_simulation_positions(list_robot_positions, list_obstacle_positions, sim_time=1)
	plot_simulation_positions(list_robot_positions, list_obstacle_positions, sim_time=10)
	plot_simulation_positions(list_robot_positions, list_obstacle_positions, sim_time=25)
	plot_simulation_positions(list_robot_positions, list_obstacle_positions, sim_time=45)

	plot_linear_velocities(linear_velocities, linear_velocities_control)
	plot_angular_velocities(angular_velocities, angular_velocities_control)
	
except KeyboardInterrupt:
    # Call the function to plot the saved graph here
    plot_simulation_positions(list_robot_positions, list_obstacle_positions, 35)
