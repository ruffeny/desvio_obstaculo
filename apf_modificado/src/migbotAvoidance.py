#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Wrench, Vector3
from ApfModifiedObstacleAvoidance import ObstacleAvoidance


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
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        self.wrench_pub = rospy.Publisher('/Wrench', Wrench, queue_size=10)

        # Definindo a taxa
        self.rate = rospy.Rate(10)  # 10 Hz

    def model_states_callback(self, data):
        for i, name in enumerate(data.name):
            if name == 'migbot':
                self.migbot.update(data.pose[i], data.twist[i])
            elif name in [ob.name for ob in self.obstacles]:
                [ob for ob in self.obstacles if ob.name == name][0].update(data.pose[i], data.twist[i])

    def apf_modified_total_force(self):
        # Aqui, chamamos a função modified_potential_field para calcular a força total
        # Por enquanto, estou retornando um valor fictício
        goal_position = [10, 10]  # Exemplo de posição do objetivo
        list_of_obstacle_positions = [ob.pose for ob in self.obstacles]
        list_of_obstacle_radii = [ob.radius for ob in self.obstacles]
        list_of_obstacle_velocities = [ob.twist for ob in self.obstacles]
        current_robot_position = self.migbot.pose
        current_robot_velocity = self.migbot.twist

        force = self.obstacle_avoidance.modified_potential_field(goal_position, list_of_obstacle_positions, list_of_obstacle_radii, list_of_obstacle_velocities, current_robot_position, current_robot_velocity)
        
        # Converta a força em um vetor e torque (a lógica de conversão precisa ser adicionada)

        # Converta a força em um vetor
        force_vector = Vector3()
        force_vector.x = total_force[0]  # Componente x da força
        force_vector.y = total_force[1]  # Componente y da força
        force_vector.z = 0.0  # Assumindo que não há força na direção z

        # Calcule a diferença angular usando o heading (ângulo theta em relação aos eixos inerciais)
        heading = self.migbot.pose[2]  # Assumindo que heading é a orientação z da pose do migbot
        angular_difference = np.arctan2(total_force[1], total_force[0]) - heading

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
                wrench_msg.force = force
                wrench_msg.torque.z = torque
                self.wrench_pub.publish(wrench_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('migbot_controller', anonymous=True)

    migbot = Migbot()
    obstacles = [Obstacle('trunk1_buoy')]  # Adicione mais obstáculos conforme necessário

    controller = MigbotController(migbot, obstacles)
    controller.run()
