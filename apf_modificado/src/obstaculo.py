#!/usr/bin/env python3

import rospy
import random
import subprocess
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
import numpy as np
from gazebo_msgs.srv import SpawnModel,SetModelState
import math
import tf
from gazebo_msgs.msg import ModelState
import roslaunch
import os
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import cmath


def save_robot_position(robot_positions,x, y):
    robot_positions.append((x, y))
    
    return robot_positions
 

def plot_robot_positions(robot_positions, obstacle_position, obstacle_radius):
    x = [pos[0] for pos in robot_positions]
    y = [pos[1] for pos in robot_positions]
    x_box = [pos[0] for pos in obstacle_position]
    y_box = [pos[1] for pos in obstacle_position]
    fig, ax = plt.subplots()
    ax.plot(x, y, 'r',x_box,y_box,'b')
    
    # plot obstacle as circle
    #obstacle_circle = Circle(obstacle_position, obstacle_radius, fill=False, edgecolor='b', linewidth=2)
    #ax.add_artist(obstacle_circle)
    # set axis limits
    ax.set_xlim([-2, 10])
    ax.set_ylim([-3, 10])
    
    plt.show()


def adjust_angle(angle):
    angle = np.mod(angle, 2 * np.pi)
    if angle > np.pi:
        angle = angle - 2 * np.pi
    if angle < -np.pi:
        angle = angle + 2 * np.pi
    angulo_ajustado = angle
    return angulo_ajustado
    
def spawn_husky_launch(husky_name, position_x=9.0, position_y=9.0, yaw=0.0):
    command = "roslaunch husky_gazebo spawn_husky.launch name:={0} x:={1} y:={2} yaw:={3}".format(husky_name, position_x, position_y, yaw)
    subprocess.call(command, shell=True)

def set_husky_velocity(husky_name, linear_x):
    pub = rospy.Publisher("/{0}/cmd_vel".format(husky_name), Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()
 
    
def modified_attractive_force(pos, vector_to_goal, attraction_scaling_factor,distance_to_goal,normalized_vector_to_goal):
    attractive_force = attraction_scaling_factor * distance_to_goal *normalized_vector_to_goal
    
    return attractive_force
 
def calculate_Fre(distance_to_obstacle, safety_margin_radius, center_to_center_safe_distance, distance_to_goal, scaling_factor_emergency, obstacle_domain_radius, unit_vector_to_obstacle, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal,perpendicular_unit_vector_to_obstacle):
	var1 = (1./(distance_to_obstacle-safety_margin_radius)-1/center_to_center_safe_distance)
	var2 = (distance_to_goal**2)/((distance_to_obstacle-safety_margin_radius)**2)
	Fre1 = -2 * scaling_factor_emergency * obstacle_domain_radius * var1 * var2 * unit_vector_to_obstacle
	Fre2 = 2 * scaling_factor_emergency * obstacle_domain_radius * distance_to_goal / distance_to_obstacle * np.linalg.norm(relative_speed_vector)**2 * (np.cos(angle_between_direction_and_velocity) * np.sin(angle_between_direction_and_velocity)) * perpendicular_unit_vector_to_obstacle
	Fre3 = 2 * scaling_factor_emergency * obstacle_domain_radius * distance_to_goal * (var1**2 + np.linalg.norm(relative_speed_vector)**2 * np.cos(angle_between_direction_and_velocity)**2) * normalized_vector_to_goal
	Fre = Fre1 + Fre2 + Fre3
	return Fre
       
def calculate_Frs(distance_to_obstacle, safety_margin_radius, distance_to_goal, obstacle_influence_range, obstacle_scaling_factor_static, obstacle_domain_radius, unit_vector_to_obstacle, normalized_vector_to_goal):
	var1 = 1/(distance_to_obstacle-safety_margin_radius) - 1/obstacle_influence_range
	var2 = (distance_to_goal**2)/(distance_to_obstacle**2)
	Frs1 = -obstacle_scaling_factor_static * obstacle_domain_radius * var1 * var2 * unit_vector_to_obstacle
	Frs3 = obstacle_scaling_factor_static * obstacle_domain_radius * distance_to_goal * var1**2 * normalized_vector_to_goal
	return Frs1 + Frs3  
    
def calculate_Frd(distance_to_obstacle,center_to_center_safe_distance,obstacle_influence_range,angle_between_direction_and_velocity,relative_speed_vector,theta_m_degree,angle_difference_for_safety,vector_to_obstacle,obstacle_scaling_factor_dynamic,obstacle_domain_radius,distance_to_goal,unit_vector_to_obstacle,perpendicular_unit_vector_to_obstacle,normalized_vector_to_goal):
	var1 = ((1./(distance_to_obstacle-center_to_center_safe_distance))-(1/obstacle_influence_range))
	var2 = (center_to_center_safe_distance/(distance_to_obstacle*np.sqrt(distance_to_obstacle**2-center_to_center_safe_distance**2)))
	var3 = (np.sin(np.radians(angle_between_direction_and_velocity))/np.linalg.norm(relative_speed_vector))
	var4 = (np.sin(np.radians(theta_m_degree))/np.linalg.norm(relative_speed_vector))
	var5 = ((np.exp(angle_difference_for_safety)-1)/((distance_to_obstacle-center_to_center_safe_distance)**2))
	Fto0 = var1*(var2 + var4)
	var6 = (1/np.linalg.norm(vector_to_obstacle)+np.cos(np.radians(angle_between_direction_and_velocity))/np.linalg.norm(relative_speed_vector))
	var7 = np.linalg.norm(relative_speed_vector)*(np.exp(angle_difference_for_safety)-1)/(distance_to_obstacle*(distance_to_obstacle-center_to_center_safe_distance)**2)
	Ftop = var1*(1/np.linalg.norm(vector_to_obstacle)+np.cos(np.radians(theta_m_degree))/np.linalg.norm(relative_speed_vector)) 
	Frd1 = -obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal**2*(var1*np.exp(angle_difference_for_safety)*(var2 + var3)+var5-Fto0)*unit_vector_to_obstacle
	Frd3 = obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal**2*var1*(np.exp(angle_difference_for_safety)-1)*normalized_vector_to_goal 
	Frd2 = -obstacle_scaling_factor_dynamic*obstacle_domain_radius*distance_to_goal**2*(var1*np.exp(angle_difference_for_safety)*var6*var7-Ftop)*perpendicular_unit_vector_to_obstacle
	Frd = Frd1 + Frd2 + Frd3
	return Frd

def modified_potential_field(goal_position,list_of_obstacle_positions,list_of_obstacle_radii,list_of_obstacle_velocities,current_robot_position,attraction_scaling_factor,obstacle_scaling_factor_dynamic,obstacle_scaling_factor_static,scaling_factor_emergency,safety_margin_radius,robot_domain_radius,safe_distance,obstacle_influence_range,current_robot_velocity):
	
	vector_to_goal = goal_position - current_robot_position 
	count = 0 #Inutilizado
	distance_to_goal = np.linalg.norm(vector_to_goal) 
	normalized_vector_to_goal = vector_to_goal/distance_to_goal

	attractive_force = np.zeros_like(current_robot_position)
	repulsive_force = np.zeros_like(current_robot_position)

	attractive_force = modified_attractive_force(current_robot_position, vector_to_goal, attraction_scaling_factor,distance_to_goal,normalized_vector_to_goal)

	for i,obstacle in enumerate(list_of_obstacle_positions):
		#initiate Frd,Frs,Fre
		Frd = np.zeros_like(current_robot_position)
		Fre = np.zeros_like(current_robot_position)
		Frs = np.zeros_like(current_robot_position)		
		numero_do_obstaculo = i

		distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius = iniciation(list_of_obstacle_positions[i], current_robot_position, current_robot_velocity, list_of_obstacle_velocities[i], list_of_obstacle_radii[i], safe_distance, obstacle_influence_range, robot_domain_radius,normalized_vector_to_goal,numero_do_obstaculo)
		
		if distance_to_obstacle <= collision_avoidance_radius and angle_between_direction_and_velocity < angle_for_safe_distance and center_to_center_safe_distance <= distance_to_obstacle:
			if not np.array_equal(list_of_obstacle_velocities[i], np.array([0, 0])):
				Frd = calculate_Frd(distance_to_obstacle,center_to_center_safe_distance,obstacle_influence_range,angle_between_direction_and_velocity,relative_speed_vector,angle_for_safe_distance,angle_difference_for_safety,vector_to_obstacle,obstacle_scaling_factor_dynamic,obstacle_domain_radius,distance_to_goal,unit_vector_to_obstacle,perpendicular_unit_vector_to_obstacle,normalized_vector_to_goal)
				print("distance_to_obstacle =",distance_to_obstacle,"< collision_avoidance_radius =",collision_avoidance_radius," dinamic --------------  Frd = ",Frd)
			else:
				Frs = calculate_Frs(distance_to_obstacle, safety_margin_radius, distance_to_goal, obstacle_influence_range, obstacle_scaling_factor_static, obstacle_domain_radius, unit_vector_to_obstacle, normalized_vector_to_goal)
				print("distance_to_obstacle =",distance_to_obstacle,"< collision_avoidance_radius =",collision_avoidance_radius," static --------------  Frs = ",Frs)
		else:
			if distance_to_obstacle < center_to_center_safe_distance :
				Fre = calculate_Fre(distance_to_obstacle, safety_margin_radius, center_to_center_safe_distance, distance_to_goal, scaling_factor_emergency, obstacle_domain_radius, unit_vector_to_obstacle, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal,perpendicular_unit_vector_to_obstacle)
				print("distance_to_obstacle =",distance_to_obstacle,"< center_to_center_safe_distance =", center_to_center_safe_distance," --------------  Fre = ",Fre)
			else:
				print(" ")
		repulsive_force += Frd + Frs + Fre
		print("obstaculo numero: ",i,"definicao do obst =",list_of_obstacle_positions[i],"velocidade do obstaculo",list_of_obstacle_velocities[i],"repulsive_force = ",repulsive_force,"angle_between_direction_and_velocity = ",angle_between_direction_and_velocity,"theta_m = ",angle_for_safe_distance)
	
	total_force = attractive_force + repulsive_force
	print("FORCA TOTAL = ",total_force)
	return total_force

def iniciation(obstacle_position, current_robot_position, current_robot_velocity, obstacle_velocitiy, obstacle_radii, safe_distance, obstacle_influence_range, robot_domain_radius,normalized_vector_to_goal,numero_do_obstaculo):
	distance_to_obstacle = np.linalg.norm(obstacle_position - np.array(current_robot_position))
	center_to_center_safe_distance = robot_domain_radius + safe_distance + obstacle_radii
	print("robot_domain_radius = ",robot_domain_radius,"current_robot_position =", current_robot_position,"obstacle_position = ",obstacle_position, "obstáculo de número: ",numero_do_obstaculo)
	#collision_avoidance_radius: The collision_avoidance_radiusitical distance from the i-th obstacle. It is calculated as the sum of center_to_center_safe_distance aobstacle_scaling_factor_dynamic obstacle_influence_range.
	collision_avoidance_radius = center_to_center_safe_distance + obstacle_influence_range
    
    #vector_to_obstacle: The vector pointing from the current position of the boat to the i-th obstacle.
	vector_to_obstacle = obstacle_position - current_robot_position
	if distance_to_obstacle >= center_to_center_safe_distance:
		angle_for_safe_distance2 = np.arctan2(center_to_center_safe_distance, np.sqrt(distance_to_obstacle**2 - center_to_center_safe_distance**2))
		angle_for_safe_distance = np.degrees(angle_for_safe_distance2)
	else:
		angle_for_safe_distance = 0
    
	relative_speed_vector = current_robot_velocity - obstacle_velocitiy
	print("VOS = ",current_robot_velocity,"VTs = ",obstacle_velocitiy)
    
    #angle_between_direction_and_velocity: The angle between the vector vector_to_obstacle aobstacle_scaling_factor_dynamic the relative velocity vector relative_speed_vector. 
	extra = 1e-8  # You can adjust this value as scaling_factor_emergencyeded
	angle_between_direction_and_velocity2 = np.arccos(np.dot(vector_to_obstacle, relative_speed_vector) / (np.linalg.norm(vector_to_obstacle) * np.linalg.norm(relative_speed_vector) + extra))
	angle_between_direction_and_velocity = np.degrees(angle_between_direction_and_velocity2)
    #angle_between_direction_and_velocity = adjust_angle(angle_between_direction_and_velocity)
    #print("angulo = ",math.degrees(angle_between_direction_and_velocity),"angulo_obst = ",math.degrees(angle_for_safe_distance) )
    
    #unit_vector_to_obstacle is a unit vector pointing from the current position to the obstacle
	unit_vector_to_obstacle = (obstacle_position - current_robot_position) / distance_to_obstacle
	angle_difference_for_safety = angle_for_safe_distance-angle_between_direction_and_velocity
	angulacao = np.degrees(np.arccos(np.dot(unit_vector_to_obstacle, normalized_vector_to_goal)/(np.linalg.norm(unit_vector_to_obstacle) * np.linalg.norm(normalized_vector_to_goal))))
	histerese = 0
	if angulacao < 5 or angulacao > 355:
		histerese = 0.1
	perpendicular_unit_vector_to_obstacle = comparar_vetores(vector_to_obstacle, relative_speed_vector, obstacle_velocitiy,unit_vector_to_obstacle,histerese)
	perpendicular_unit_vector_to_obstacle = decide_rotacao(current_robot_velocity,obstacle_velocitiy,unit_vector_to_obstacle)
	#perpendicular_unit_vector_to_obstacle = determiscaling_factor_emergency_side(vector_to_obstacle, relative_speed_vector,unit_vector_to_obstacle)
	obstacle_domain_radius = obstacle_radii
	
	return distance_to_obstacle, center_to_center_safe_distance, collision_avoidance_radius, vector_to_obstacle, angle_for_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, unit_vector_to_obstacle, angle_difference_for_safety, perpendicular_unit_vector_to_obstacle, obstacle_domain_radius

def comparar_vetores(v1, v2, v3,Not,histerese): #Pot, Vto, obstacles_velocities,Not,histerese
    
    # calcula o produto vetorial entre os vetores v1 e v2, e entre os vetores v3 e v2
    produto_v1_v2 = v1[0]*v2[1] - v1[1]*v2[0]
    produto_v3_v2 = v3[0]*v2[1] - v3[1]*v2[0]

    # verifica a posição de v1 em relação a v2
    if produto_v1_v2 > histerese:  # v1 está à esquerda de v2
        # verifica a posição de v3 em relação a v2
        if produto_v3_v2 > histerese:  # v3 está à direita de v2
            return np.array([Not[1], -Not[0]])  # Rotate counterclockwise#"obstáculo à esquerda, mas andando para direita"
        elif produto_v3_v2 <= -histerese:  # v3 está à esquerda de v2
            return np.array([-Not[1], Not[0]])  # Rotate clockwise#"obstáculo à esquerda e obstáculo indo para a esquerda também"
    elif produto_v1_v2 < -histerese:  # v1 está à direita de v2
        # verifica a posição de v3 em relação a v2
        if produto_v3_v2 <= histerese:  # v3 está à esquerda de v2
            return np.array([-Not[1], Not[0]])  # Rotate clockwise#"obstáculo à direita, mas andando para esquerda"
        elif produto_v3_v2 > -histerese:  # v3 está à direita de v2
            return np.array([Not[1], -Not[0]])  # Rotate counterclockwise#"obstáculo à direita e andando para direita"

def decide_rotacao(vr, vo,Not):
    """
    Decide a direção da rotação para desviar do obstáculo.

    :param vr: tuple (vrx, vry), vetor velocidade do robô
    :param vo: tuple (vox, voy), vetor velocidade do obstáculo
    :return: string, "anti-horário", "horário" ou "paralelo"
    """
    vrx, vry = vr
    vox, voy = vo

    # Calcula a componente z do produto vetorial
    pz = vrx * voy - vry * vox
    
    # Calcula o produto escalar e as magnitudes dos vetores de velocidade
    dot_product = vrx * vox + vry * voy
    vr_magnitude = math.sqrt(vrx ** 2 + vry ** 2)
    vo_magnitude = math.sqrt(vox ** 2 + voy ** 2)

    # Calcula o ângulo entre os vetores de velocidade em graus
    angle_rad = math.acos(dot_product / (vr_magnitude * vo_magnitude))
    angle_deg = math.degrees(angle_rad)

    # Se o ângulo estiver entre 175 e 185 graus, escolha uma direção de rotação padrão (anti-horário, por exemplo)
    if 165 <= angle_deg <= 195:
        return -np.array([Not[1], -Not[0]])  #"anti-horário"
    elif pz > 0:
        return -np.array([Not[1], -Not[0]])  #"anti-horário"
    elif pz < 0:
        return -np.array([-Not[1], Not[0]]) #"horário"
    else:
        return -np.array([Not[1], -Not[0]]) #"paralelo"  


    # caso não tenha retornado ainda, não há comparação possível
    #print("Não há comparação possível")
    return np.array([-Not[1], Not[0]])
    
def determine_side(v1, v2, Not):
    # Calculate the unit vector pointing along v1
    v1_norm = np.linalg.norm(v1)
    v1_unit = v1 / v1_norm if v1_norm > 0 else np.array([0, 0])
    
    # Calculate the unit vector pointing along v2
    v2_norm = np.linalg.norm(v2)
    v2_unit = v2 / v2_norm if v2_norm > 0 else np.array([0, 0])
    
    # Calculate the dot product of the two unit vectors
    dot_product = np.dot(v1_unit, v2_unit)
    
    # Determine the orientation of v2 relative to v1
    if dot_product > 0:
        # v2 is to the right of v1 or collinear with it
        return np.array([-Not[1], Not[0]])  # Rotate clockwise
    else:
        # v2 is to the left of v1
        return np.array([Not[1], -Not[0]])  # Rotate counterclockwise
        

def force_to_velocities(total_force, heading):
    # Calculate the linear velocity as the magnitude of the total force
    linear_velocity = np.linalg.norm(total_force)
    
    # Calculate the angular velocity as the angle between the heading and the direction of the total force
    angular_velocity = (np.arctan2(total_force[1], total_force[0]) - (heading))
    #angular_velocity = np.arctan2(total_force[1].real, total_force[0].real) - heading
    
    return linear_velocity, angular_velocity
    
def change_position(name,pos):
	rospy.wait_for_service('/gazebo/set_model_state')
	set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	model_state = ModelState()
	model_state.model_name = name
	model_state.pose.position.x = pos[0]
	model_state.pose.position.y = pos[1]
	model_state.pose.orientation = Quaternion(0, 0, math.radians(40), 1)
	
	set_model_state(model_state)

def change_position_velocity(name,pos,vel):
	rospy.wait_for_service('/gazebo/set_model_state')
	set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	model_state = ModelState()
	model_state.model_name = name
	model_state.pose.position.x = pos[0]
	model_state.pose.position.y = pos[1]
	model_state.twist.linear.x = vel[0]
	model_state.twist.linear.y = vel[1]
	model_state.pose.orientation = Quaternion(0, 0, 0, 1)
	
	set_model_state(model_state)
def spawn_obstacle():
    package = 'gazebo_ros'
    executable = 'spawn_model'
    node = roslaunch.core.Node(package, executable)
    node.args = "-urdf -param /husky_description -model obst_1 -x 1 -y 1"

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    rospy.loginfo("Spawned Husky robot with name 'obst_1' and position (1, 1)")
    

    

