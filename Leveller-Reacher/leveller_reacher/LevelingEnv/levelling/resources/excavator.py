import pybullet as p
import os
import math
import time
import numpy as np

class excavator:
    def __init__(self, client):
        self.client = client

        self.excavator = p.loadURDF("excavator.urdf")

        self.joint1 = [1]
        self.joint2 = [2]
        self.joint3 = [3]



    def apply_action(self, action):

        joint1, joint2, joint3 = action
        x = 1
        joint1 = min(max(joint1, -x), x)
        joint2 = min(max(joint2, -x), x)
        joint3 = min(max(joint3, -x), x)
        
        p.setJointMotorControlArray(self.excavator, self.joint1,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocities=[joint1],
                                    physicsClientId=self.client)

        p.setJointMotorControlArray(self.excavator, self.joint2,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocities=[joint2],
                                    physicsClientId=self.client)

        p.setJointMotorControlArray(self.excavator, self.joint3,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocities=[joint3],
                                    physicsClientId=self.client)


    def get_observation(self, y1, z1, y2, z2, goal_switch):
        
        pos1, ang1 = p.getLinkState(self.excavator, 1)[:2]
        ang1 = p.getEulerFromQuaternion(ang1)
        ori1 = ang1[0]

        pos2, ang2 = p.getLinkState(self.excavator, 2)[:2]
        ang1 = p.getEulerFromQuaternion(ang2)
        ori2 = ang2[0]

        pos3, ang3 = p.getLinkState(self.excavator, 3)[:2]
        ang3 = p.getEulerFromQuaternion(ang3)
        ori3 = ang3[0]
        
        pos4, ang4 = p.getLinkState(self.excavator, 4)[:2]
        ang4 = p.getEulerFromQuaternion(ang4)
        ori4 = ang4[0]
        pos4y = pos4[1]
        pos4z = pos4[2]

        observation = np.array([pos4y, pos4z, ori4, ori3, ori2, ori1, y1, z1, y2, z2, goal_switch])
        
               
        return observation







