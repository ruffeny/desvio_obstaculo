#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
import numpy as np
from numpy.linalg import inv
import math
from geometry_msgs.msg import Vector3

def getOdom(msg):
	global x, y, theta, v_gazebo, w_gazebo
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y	
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
	v_gazebo.x = msg.twist.twist.linear.x
	v_gazebo.y = msg.twist.twist.linear.y
	w_gazebo.z = msg.twist.twist.angular.z	
	#print("x = ",x," e y = ",y, "velocity = ",v_gazebo,"angular velocity = ",w_gazebo)

def modified_attractive_force(pos, pg, epsilon,dg,Nog):
    #calculates the attractive force between a target position "pg" and a current position "pos". The function first calculates the direction 	vector "nog" between the two positions
    
    
    #calculate the Atractive force 
    Fatt = epsilon * dg *Nog
    
    return Fatt
 
def calculate_Fre(d, tau, Dm, dg, Ne, Rts, Not, Vto, tetha, Nog,Notperp):
	var1 = (1./(d-tau)-1/Dm)
	var2 = (dg**2)/((d-tau)**2)
	Fre1 = -2 * Ne * Rts * var1 * var2 * Not
	Fre2 = 2 * Ne * Rts * dg / d * np.linalg.norm(Vto)**2 * (np.cos(np.radians(tetha)) * np.sin(np.radians(tetha))) * Notperp
	Fre3 = 2 * Ne * Rts * dg * (var1**2 + np.linalg.norm(Vto)**2 * np.cos(np.radians(tetha))**2) * Nog
	Fre = Fre1 + Fre2 + Fre3
	return Fre
       
def calculate_Frs(d, tau, dg, phou0, Ns, Rts, Not, Nog):
 	var1 = 1./(d-tau) - 1/phou0
 	var2 = (dg**2)/(d**2)
 	Frs1 = -Ns * Rts * var1 * var2 * Not
 	Frs3 = Ns * Rts * dg * var1**2 * Nog
 	return Frs1 + Frs3  
    
def calculate_Frd(d,Dm,phou0,tetha,Vto,theta_m_degree,AngleDiff,Pot,Nd,Rts,dg,Not,Notperp,Nog):
	var1 = ((1./(d-Dm))-(1/phou0))
	var2 = (Dm/(d*np.sqrt(d**2-Dm**2)))
	var3 = (np.sin(np.radians(tetha))/np.linalg.norm(Vto))
	var4 = (np.sin(np.radians(theta_m_degree))/np.linalg.norm(Vto))
	var5 = ((np.exp(AngleDiff)-1)/((d-Dm)**2))
	Fto0 = var1*(var2 + var4)
	var6 = (1/np.linalg.norm(Pot)+np.cos(np.radians(tetha))/np.linalg.norm(Vto))
	var7 = np.linalg.norm(Vto)*(np.exp(AngleDiff)-1)/(d*(d-Dm)**2)
	Ftop = var1*(1/np.linalg.norm(Pot)+np.cos(np.radians(theta_m_degree))/np.linalg.norm(Vto)) 
	Frd1 = -Nd*Rts*dg**2*(var1*np.exp(AngleDiff)*(var2 + var3)+var5-Fto0)*Not
	Frd3 = Nd*Rts*dg**2*var1*(np.exp(AngleDiff)-1)*Nog 
	Frd2 = Nd*Rts*dg**2*(var1*np.exp(AngleDiff)*var6*var7-Ftop)*Notperp
	Frd = Frd1 + Frd2 + Frd3
	return Frd

def modified_potential_field(goal,obstacles,obstacles_radius,obstacles_velocities,current_position,epsilon,Nd,Ns,Ne,tau,Ros,Dsafe,phou0,vos_velocity, Dm,CR):
	#vector that point from the robot_position to the goal
	pg = goal - current_position 		
	
	#the vector's size is the distance until the goal.
	dg = np.linalg.norm(pg) 
	
	#Unit vector in goal direction
	Nog = pg/dg  
	
	#calculate the attractive force	
	Fatt = modified_attractive_force(current_position, pg, epsilon,dg,Nog)
	
	# Initialize the repulsive force
	Frep = np.zeros_like(current_position)
    	
    		
    	# Calculate the repulsive force from each obstacle
	for i,obstacle in enumerate(obstacles):
		#initiate Frd,Frs,Fre
		Frd = np.zeros_like(current_position)
		Fre = np.zeros_like(current_position)
		Frs = np.zeros_like(current_position)
		
		#get some parameters related to the obstacle i
		d, Pot, theta_m_radian, Vto, tetha, Not, AngleDiff, Notperp, Rts = iniciation(obstacles[i], current_position, vos_velocity, obstacles_velocities[i], obstacles_radius[i], Dsafe, phou0)
		
		if d <= CR and tetha < theta_m_radian and Dm <= d:
    			if obstacles_velocities[i][0] != 0 or obstacles_velocities[i][1] != 0:
        			Frd = calculate_Frd(d,Dm,phou0,tetha,Vto,theta_m_radian,AngleDiff,Pot,Nd,Rts,dg,Not,Notperp,Nog)
    			else:
        			Frs = calculate_Frs(d, tau, dg, phou0, Ns, Rts, Not, Nog)
		else:
    			if d <= Dm:
        			Fre = calculate_Fre(d, tau, Dm, dg, Ne, Rts, Not, Vto, tetha, Nog,Notperp)
    			else:
        			#no obstacles
        			pass	
		
		Frep += Frd + Frs + Fre
        
    	# Return the sum of the attractive and repulsive forces as the total force
	total_force = Fatt + Frep
	return total_force


def iniciation(obstacles, current_position, vos_velocity, obstacles_velocities, obstacles_radius, Dsafe, phou0):
    #d: Euclidean distance between the current position of the boat and the i-th obstacle.
    d = np.linalg.norm(obstacles - current_position)
    
    #Pot: The vector pointing from the current position of the boat to the i-th obstacle.
    Pot = obstacles - current_position 
    
    #theta_m_radian: The angle between the vector Pot and a vector pointing to the direction of the normal line passing through the point on the circumference of the obstacle at a safe distance from it.
    theta_m_radian = np.arctan(Dm/np.sqrt(d**2 - Dm**2))
    
    #vto: The relative velocity between the boat and the i-th obstacle. 
    Vto = vos_velocity - obstacles_velocities
    
    #tetha: The angle between the vector Pot and the relative velocity vector vto.
    tetha = np.arccos(np.dot(Pot, Vto) / (np.linalg.norm(Pot) * np.linalg.norm(Vto)))
    
    #Not is a unit vector pointing from the current position to the obstacle
    Not = (obstacles - current_position) / d
    
    AngleDiff = theta_m_radian-tetha
    Notperp = determine_side(Vto, Pot, Not)
    Rts = obstacles_radius
    
    return d, Pot, theta_m_radian, Vto, tetha, Not, AngleDiff, Notperp, Rts

def determine_side(v1, v2, Not):
    """
    This function determines on which side of the "v1" vector the "v2" vector lies.
    If v2 is on the left side of v1, a unit vector perpendicular to and clockwise of the "Not" vector is calculated.
    If v2 is on the right side of v1, a unit vector perpendicular to and counterclockwise of the "Not" vector is calculated.
    If v1 and v2 are collinear, a unit vector perpendicular to and clockwise of the "Not" vector is calculated.
    :param v1: A 2D vector
    :param v2: A 2D vector
    :param Not: A 2D vector
    :return: A 2D unit vector perpendicular to Not in the direction determined by the position of v2 relative to v1
    """
    # Normalize the vectors v1 and v2 to unit vectors v1 = Vto v2 = Pot
    AB = v1 / np.linalg.norm(v1) 
    AP = v2 / np.linalg.norm(v2)

    # Calculate the dot product of the unit vector AB and the unit vector AP
    result = np.dot(AB, AP)

    # Determine the side of v2 relative to v1
    if result > 0:  # v2 is on the right side of v1
        lado = 1
    elif result < 0:  # v2 is on the left side of v1
        lado = 0
    else:  # v1 and v2 are collinear
        lado = None

    # Calculate the unit vector perpendicular to Not in the direction determined by the position of v2 relative to v1
    if lado == 0:  # v2 is on the left side of v1
        Notperp = np.array([-Not[1], Not[0]])  # rotate Not 90 degrees counterclockwise
    elif lado == 1:  # v2 is on the right side of v1
        Notperp = np.array([Not[1], -Not[0]])  # rotate Not 90 degrees clockwise
    else:  # v1 and v2 are collinear
        Notperp = np.array([-Not[1], Not[0]])  # rotate Not 90 degrees counterclockwise

    # Normalize the calculated vector to a unit vector
    Notperp = Notperp / np.linalg.norm(Notperp)

    return Notperp

def force_to_velocities(total_force, heading):
    # Calculate the linear velocity as the magnitude of the total force
    linear_velocity = np.linalg.norm(total_force)
    
    # Calculate the angular velocity as the angle between the heading and the direction of the total force
    angular_velocity = np.arctan2(total_force[1], total_force[0]) - heading
    
    return linear_velocity, angular_velocity
    

    	
x = 0.0
y = 0.0
v_gazebo = Vector3()
w_gazebo = Vector3()
theta = 0.0

#define the obstacle position
obstacles = [np.array([2, 2]), np.array([6, 2]), np.array([2, 6]), np.array([6, 6])]

#define the obstacle radius
obstacles_radius = np.array([0.5, 0.5, 0.5, 0.5])

#define the obstcale velocities
obstacles_velocities = [np.array([0, 0]), np.array([0, 0]), np.array([0, 0]), np.array([0, 0])]

#APF  Modificado - Parâmetros
epsilon = 30000;#3000
Nd = 2000;#2000 velocidade diferente de zero
Ns = 300000;#300000  velocidade isgual de zero
Ne = 2000;#2000  emergencia
tau = 0.3; #mn milha nautica = 1852 metros
Ros = 0.5; #mn
Dsafe = 0.5; #0.5mn ou 1 se for mar aberto
phou0 = 3; #3 ate 5mn
maxturn = 90 ; #5 graus
timeStep = 15; #segundos
Rts = 0.3; #mn
vmax = 0.4; #m/s
wmax = math.radians(maxturn);
Rts = np.amax(obstacles_radius)
Dm = Ros + Dsafe + Rts;
CR = Dm + phou0;
#force = np.array([0.1, 0.2, 0.3])
husky_mass = 46.03
husky_inertia = np.array([[0.06, -0.02, -0.12], [1.74, 0, 2.03],[0,0,0.1]])


rospy.init_node("test_controller")
sub = rospy.Subscriber("/odom_husky", Odometry, getOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
speed = Twist()

r = rospy.Rate(20) # 2 hz


goal = Point()	
goal.x = 9
goal.y = 9
goal_pos = np.array([goal.x, goal.y])



while not rospy.is_shutdown():
	# Define the goal position and the current position of the ship
	robot_pos = np.array([x, y])
	
	#own ship velocity
	vos_velocity = np.array([v_gazebo.x, v_gazebo.y])
	
	# Define the positions of the obstacles
	
	# Calculate the total force acting on the ship
	total_force = modified_potential_field(goal_pos,obstacles,obstacles_radius,obstacles_velocities,robot_pos,epsilon,Nd,Ns,Ne,tau,Ros,Dsafe,phou0,vos_velocity,Dm,CR)
	
	# Print the result
	#print("Total Force:", total_force)
	
	#transform force to linear and angular velocities	
	v,w = force_to_velocities(total_force, theta)
	
	#print("posicao do robo =",robot_pos,"goal pos = ",goal_pos,"forca total = ",total_force)
	#print("v = ",v,"w = ",w)
	
	speed.linear.x = min(v, vmax)
	speed.angular.z = np.sign(w) * min(abs(w), wmax)
	#print("speed = ",speed)
	print("v = ",speed.linear.x," w = ",speed.angular.z)

	distance_to_goal = np.linalg.norm(goal_pos - robot_pos) 
	if distance_to_goal < 0.5:
		speed.linear.x = 0
		speed.angular.z = 0	
	pub.publish(speed)
	r.sleep()
	
