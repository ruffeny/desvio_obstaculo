#!/usr/bin/env python3


import math
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Wrench, Vector3, Twist
from ApfModifiedObstacleAvoidance import ObstacleAvoidance
import numpy as np
from tf.transformations import euler_from_quaternion

# Definindo constantes 
ROBOT_NAME = 'husky'  #Modelo a ser controlado
OBSTACLE_NAMES = ['Construction Cone_0', 'Construction Barrel_0']  # Lista de nomes dos obstáculos, adicione mais conforme necessário
TOPIC_SUB = "/gazebo/model_states" #Topico do gazebo para pegar Pose e Twist dos models
TOPIC_PUB_TWIST = '/husky_velocity_controller/cmd_vel' #Topico para ser enviado o sinal de controle em velocidade 
TOPIC_PUB_WRENCH = '/Wrench' #Topico para enviar o sinal de controle em força
MAX_LINEAR_SPEED = 1.0 #Maxima velocidade linear do robot
GOAL_DEFINITION = [0, 0] #Posicao que se deseja chegar
DISTANCE_THRESHOLD = 0.5 #Distancia para parar

ATTRACTION_SCALING_FACTOR = 6000        #ajusta a magnitude da força atrativa
OBSTACLE_SCALING_FACTOR_DYNAMIC = 20000  # fatores de escala para os obstáculos dinâmicos
OBSTACLE_SCALING_FACTOR_STATIC = 3000000 #fatores de escala para os obstáculos estáticos
SCALING_FACTOR_EMERGENCY = 40000 # fator de escala para ações de evasão de emergência em relação a obstáculos próximos
SAFETY_MARGIN_RADIUS = 0.3 #raio de uma margem de segurança artificial para o robô
ROBOT_DOMAIN_RADIUS = 0.5       #Raio do robo
SAFE_DISTANCE =1 #distância segura permissível entre o robô e um obstáculo
OBSTACLE_INFLUENCE_RANGE = 5 #o alcance de influência do conjunto de obstáculos

class Migbot:
    def __init__(self):
        self.pose = None
        self.twist = None

    def update(self, pose, twist):
        self.pose = pose
        self.twist = twist

class Obstacle:
    def __init__(self, name, radius=0.5):
        self.name = name
        self.pose = None
        self.twist = None
        self.radius = radius

    def update(self, pose, twist):
        self.pose = pose
        self.twist = twist

class MigbotController:
    def __init__(self, migbot, obstacles):
        self.migbot = migbot
        self.obstacles = obstacles
        self.obstacle_avoidance = ObstacleAvoidance()  # Instância da classe ObstacleAvoidance

        # Subscribers e Publishers
        rospy.Subscriber(TOPIC_SUB, ModelStates, self.model_states_callback)
        self.wrench_pub = rospy.Publisher(TOPIC_PUB_WRENCH, Wrench, queue_size=10)
        self.Twist_pub = rospy.Publisher(TOPIC_PUB_TWIST, Twist, queue_size=10)

        # Definindo a taxa
        self.rate = rospy.Rate(10)  # 10 Hz

    def model_states_callback(self, data):
        for i, name in enumerate(data.name):
            if name == ROBOT_NAME:
                self.migbot.update(data.pose[i], data.twist[i])
                # Logando as informações do migbot
                #rospy.loginfo(f"Migbot Position: {data.pose[i].position}")
                #rospy.loginfo(f"Migbot Velocity: {data.twist[i].linear}")
            elif name in [ob.name for ob in self.obstacles]:
                [ob for ob in self.obstacles if ob.name == name][0].update(data.pose[i], data.twist[i])
                # Logando as informações do obstáculo
                #rospy.loginfo(f"Obstacle {name} Position: {data.pose[i].position}")
                #rospy.loginfo(f"Obstacle {name} Velocity: {data.twist[i].linear}")

    def apf_modified_total_force(self):
        # Aqui, chamamos a função modified_potential_field para calcular a força total
        # Por enquanto, estou retornando um valor fictício
        goal_position = GOAL_DEFINITION  # Exemplo de posição do objetivo
        list_of_obstacle_positions = [[ob.pose.position.x, ob.pose.position.y] for ob in self.obstacles]
        list_of_obstacle_radii = [ob.radius for ob in self.obstacles]
        list_of_obstacle_velocities = [[ob.twist.linear.x, ob.twist.linear.y] for ob in self.obstacles]

        # Convertendo a posição e a velocidade do robô para uma lista
        current_robot_position = [self.migbot.pose.position.x, self.migbot.pose.position.y]
        current_robot_velocity = [self.migbot.twist.linear.x, self.migbot.twist.linear.y]

        #current_robot_position = self.migbot.pose
        #current_robot_velocity = self.migbot.twist

        total_force = self.obstacle_avoidance.modified_potential_field(goal_position, list_of_obstacle_positions, list_of_obstacle_radii, list_of_obstacle_velocities, current_robot_position, current_robot_velocity)
        
        # Converta a força em um vetor e torque (a lógica de conversão precisa ser adicionada)

        # Converta a força em um vetor
        force_vector = Vector3()
        force_vector.x = abs(total_force[0])  # Componente x da força
        force_vector.y = 0.0 #total_force[1]  # Componente y da força
        force_vector.z = 0.0  # Assumindo que não há força na direção z

        # Calcular a distância ao objetivo
        distance_to_goal = math.sqrt(
            (current_robot_position[0] - goal_position[0]) ** 2 +
            (current_robot_position[1] - goal_position[1]) ** 2
        )

        # Verificar se a distância ao objetivo é menor que um limiar
        if distance_to_goal < DISTANCE_THRESHOLD:
            # Robô atingiu o objetivo, então pare o robô
            force_vector = Vector3(0.0, 0.0, 0.0)
            torque_vector = Vector3(0.0, 0.0, 0.0)

        # Convertendo o quaternião para ângulos de Euler
    
        orientation_q = self.migbot.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # Agora a variável yaw contém o ângulo de rotação ao redor do eixo z
        heading = yaw
        

        # Certifique-se de que ambos os ângulos estão no intervalo [-pi, pi]
        angle_force = np.arctan2(total_force[1], total_force[0])
        angle_force = (angle_force + np.pi) % (2 * np.pi) - np.pi
        angle_force_degrees = math.degrees(angle_force)

        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        yaw_degrees = math.degrees(yaw)

        # Calcule a diferença angular e ajuste para o intervalo [-pi, pi]
        angular_difference = angle_force - yaw
        angular_difference = (angular_difference + np.pi) % (2 * np.pi) - np.pi
        angular_difference_degree = math.degrees(angular_difference)  



        #rospy.loginfo(f"yaw: { yaw_degrees }  graus")
        #rospy.loginfo(f"angle_force: { angle_force_degrees }  graus")
        #rospy.loginfo(f"force_total: { total_force }  N")
        #angular_difference = np.arctan2(total_force[1], total_force[0]) - heading


        

        # Converta essa diferença angular em torque
        # Por simplicidade, vamos assumir um fator de proporção de 1.0
        proportion_factor = 1.0
        torque_value = proportion_factor * angular_difference

        torque_vector = Vector3()
        torque_vector.x = 0.0
        torque_vector.y = 0.0
        torque_vector.z = torque_value  # O torque será aplicado na direção z


        return force_vector, torque_vector

    def run(self):
        while not rospy.is_shutdown():
            if self.migbot.pose and self.migbot.twist:
                force, torque = self.apf_modified_total_force()
                wrench_msg = Wrench()
                Twist_msg = Twist()
                #rospy.loginfo(f"heading: {self.obstacle_avoidance.getDistanceToGoal():.2f}")
                
                wrench_msg.force = force
                wrench_msg.torque.z = torque
                Twist_msg.linear = force
                Twist_msg.angular.z = torque.z


                # Calculando a velocidade linear
                linear_speed = math.sqrt(Twist_msg.linear.x**2 + Twist_msg.linear.y**2 + Twist_msg.linear.z**2)

                # Normalizando a velocidade linear para garantir que não ultrapasse a velocidade máxima
                if linear_speed > MAX_LINEAR_SPEED:
                    normalization_factor = MAX_LINEAR_SPEED / linear_speed
                    Twist_msg.linear.x *= normalization_factor
                    Twist_msg.linear.y *= normalization_factor
                    Twist_msg.linear.z *= normalization_factor
                    linear_speed = MAX_LINEAR_SPEED  # Agora a velocidade linear é igual a velocidade máxima

                
                self.Twist_pub.publish(Twist_msg)            
                self.wrench_pub.publish(wrench_msg)  
                #rospy.loginfo(f"Velocidade Linear: {linear_speed:.2f} m/s")


            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('migbot_controller', anonymous=True)
    #rospy.loginfo("Node iniciated.")
    migbot = Migbot()
    obstacles = [Obstacle(name) for name in OBSTACLE_NAMES]  # Usando a constante aqui

    controller = MigbotController(migbot, obstacles)
    controller.obstacle_avoidance.set_parameters(
        attraction_scaling_factor=ATTRACTION_SCALING_FACTOR, 
        obstacle_scaling_factor_dynamic= OBSTACLE_SCALING_FACTOR_DYNAMIC,
        obstacle_scaling_factor_static = OBSTACLE_SCALING_FACTOR_STATIC,
        scaling_factor_emergency = SCALING_FACTOR_EMERGENCY,
        safety_margin_radius = SAFETY_MARGIN_RADIUS,
        robot_domain_radius = ROBOT_DOMAIN_RADIUS,        
        safe_distance=SAFE_DISTANCE,
        obstacle_influence_range = OBSTACLE_INFLUENCE_RANGE,
    )

    controller.run()
