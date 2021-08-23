import gym
import numpy as np
import math
import pybullet as p
import time
import pybullet_data
from levelling.resources.excavator import excavator
import matplotlib.pyplot as plt
import random



class LevellingPyBulletEnv(gym.Env):

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.array([-10, -10, -10], dtype=np.float32),
            high=np.array([10, 10, 10], dtype=np.float32))
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-1, -4, -4, -3.14, -3.14, -3.14, -1, -4, -1, -4, 0], dtype=np.float32),
            high=np.array([4, 4, 4, 3.14, 3.14, 3.14, 4, 4, 4, 4, 2], dtype=np.float32))

        self.client = p.connect(p.GUI)

        self.excavator = None
        self.done = False
        global goal_switch
        goal_switch = 0

        self.reset()

    def step(self, action, c):
        reward = 0
        action = action
        self.excavator.apply_action(action)
        
        p.stepSimulation()

        global goal_switch


        excavator_ob = self.excavator.get_observation(y1, z1, y2, z2, goal_switch)
        
        
        potential1 = math.sqrt(((excavator_ob[0] - y1) ** 2 +
                                  (excavator_ob[1] - z1) ** 2))

        potential2 = math.sqrt(((excavator_ob[0] - y2) ** 2 +
                                  (excavator_ob[1] - z2) ** 2))



        m = (z2-z1)/(y2-y1)
        func = m*(excavator_ob[0]-y1)+z1



        if goal_switch == 0:
            reward = self.potential_old1 - potential1

        if potential1 < 0.05 and goal_switch == 0:
            goal_switch = 1

        if goal_switch > 0:
            reward = (self.potential_old2-potential2)/((abs(excavator_ob[1]-func))+0.1)

    
        self.potential_old1 = potential1
        self.potential_old2 = potential2

        ob = np.array(excavator_ob, dtype=np.float32)

        return (ob, reward, self.done, c, dict())
    

    
    def reset(self):
        p.resetSimulation(self.client)
        self.done = False
        p.setGravity(0, 0, -9.81)
        self.excavator = excavator(self.client)

        global y1, y2, z1, z2, y1a, z1a
        y1 = random.uniform(0.4, 1.5)
        z1 = random.uniform(0.1, 1)
        y2 = random.uniform(1.7, 2.1)
        z2 = random.uniform(0.1, 1)
        global goal_switch
        goal_switch = 0
        y1a = y1
        z1a = z1
        p.loadURDF("plane.urdf")
        goal1 = p.loadURDF("goal1.urdf", basePosition=[0, y1, z1])
        goal2 = p.loadURDF("goal2.urdf", basePosition=[0, y2, z2])

        excavator_ob = self.excavator.get_observation(y1, z1, y2, z2, goal_switch)
        self.potential_old1 = math.sqrt(((excavator_ob[0] - y1) ** 2 +
                                           (excavator_ob[1] - z1) ** 2))

        self.potential_old2 = math.sqrt(((excavator_ob[0] - y2) ** 2 +
                                             (excavator_ob[1] - z2)**2))

        return np.array(excavator_ob + y1+ z1 + y2 + z2 + y1a + z1a, dtype=np.float32)

    def render(self, mode='human', close=False):
        pass



    def close(self):
        p.disconnect(self.client)


