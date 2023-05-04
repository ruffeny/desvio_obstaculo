
import numpy as np

def predefined_scenarios(scenario_name):
    scenarios = {
        'scenario_1': { #um obstaculo estatico
            'goal_pos': np.array([10, 10]),
            'obstacle_positions': [np.array([4,4]),np.array([-3,-9]),np.array([-6,-6]),np.array([-9,-3]),np.array([-8,-8]),np.array([-9,-9])],
            'obstacle_velocities': [np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0])],
            'obstacle_radii': [0.4, 0.4, 0.4, 0.25, 0.25, 0.25],
        },
        'scenario_2': { #dois obstaculos estaticos
            'goal_pos': np.array([10, 10]),
            'obstacle_positions': [np.array([5,6]),np.array([7,3]),np.array([-6,-6]),np.array([-9,-3]),np.array([-8,-8]),np.array([-9,-9])],
            'obstacle_velocities': [np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0])],
            'obstacle_radii': [0.4, 0.4, 0.4, 0.25, 0.25, 0.25],
        },
        'scenario_3': {  #obstaculo movel saindo do goal position e indo na direcao do husky
            'goal_pos': np.array([10, 10]),
            'obstacle_positions': [np.array([-8,-8]),np.array([-7,1]),np.array([-6,-6]),np.array([-9,-3]),np.array([7,7.5]),np.array([-9,-9])],
            'obstacle_velocities': [np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([-0.1,-0.1]),np.array([0,0])],
            'obstacle_radii': [0.4, 0.4, 0.4, 0.25, 0.25, 0.25],
        },
        'scenario_4': {  #obstaculo movel encontro vindo a direito do husky ( deve virar para direita para evitar o obstaculo)
            'goal_pos': np.array([10, 10]),
            'obstacle_positions': [np.array([8,0]),np.array([-8,0]),np.array([-6,-6]),np.array([-9,-3]),np.array([-8,-8]),np.array([-9,-9])],
            'obstacle_velocities': [np.array([-0.135, 0.27]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0])],
            'obstacle_radii': [0.4, 0.4, 0.4, 0.25, 0.25, 0.25],
        },
        'scenario_5': {  #obstaculo movel encontro vindo da esquerda do husky ( deve virar para esquerda para evitar o obstaculo)
            'goal_pos': np.array([10, 10]),
            'obstacle_positions': [np.array([0,8]),np.array([-8,0]),np.array([-6,-6]),np.array([-9,-3]),np.array([-8,-8]),np.array([-9,-9])],
            'obstacle_velocities': [np.array([0.22,-0.195]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0])],
            'obstacle_radii': [0.4, 0.4, 0.4, 0.25, 0.25, 0.25],
        },
        'scenario_6': {  #4 obstaculos estaticos
            'goal_pos': np.array([10, 10]),
            'obstacle_positions': [np.array([4,0]),np.array([2,2]),np.array([6,2]),np.array([6,6]),np.array([-8,-8]),np.array([-9,-9])],
            'obstacle_velocities': [np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0]),np.array([0,0])],
            'obstacle_radii': [0.4, 0.4, 0.4, 0.25, 0.25, 0.25],
        },
        'scenario_7': {  #obstaculos do artigo
            'goal_pos': np.array([10, 10]),
            'obstacle_positions': [np.array([7.8,2.2]),np.array([6.8,4.9]),np.array([7,0]),np.array([7.5,7]),np.array([6,8]),np.array([4,8])],
            'obstacle_velocities': [np.array([0,0]),np.array([0,0]),np.array([-0.2,0.2]),np.array([-0.18,-0.18]),np.array([0,-1/15]),np.array([2.8/20,-1.6/20])],
            'obstacle_radii': [0.4, 0.4, 0.4, 0.25, 0.25, 0.25],
        },	
        # ...
    }
    return scenarios.get(scenario_name)
