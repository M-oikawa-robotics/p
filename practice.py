import pybullet
import time
import pybullet_data
import PIL
from PIL import Image
import numpy as np
import mymod
import chainer
import datetime, time, random, csv, math
import matplotlib.pylab as plt
from chainer import Chain, Variable, cuda, optimizer, optimizers, serializers
import chainer.functions as F
import chainer.links as L

dT = 0.1 * 1e-3
f_ctrl = 10
theta = []
trq = []


physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
#physicsClient = pybullet.connect(pybullet.DIRECT)  # or pybullet.DIRECT for non-graphical version
pybullet.setTimeStep(dT)

pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
pybullet.setGravity(0, 0, -9.8)
planeId = pybullet.loadURDF("plane.urdf")
cubeStartPos = [0.0, 0.0, 0]
cubeStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])
boxId = pybullet.loadURDF("./URDF/motoman-mh3f.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=1)
#pancakeId = pybullet.loadURDF("./URDF/samurai.urdf", [0.4, 0.0, 0.0], pybullet.getQuaternionFromEuler([0, 0, 0]), useFixedBase=1)
pancakeId = pybullet.loadURDF("./URDF/samurai_hole.urdf", [0.47, 0.02, 0.0], pybullet.getQuaternionFromEuler([0, 0, 180*3.14/360]), useFixedBase=1)
#pancakeId = pybullet.loadURDF("./URDF/samurai_hole.urdf", [0.48, 0.0, 0.0], pybullet.getQuaternionFromEuler([0, -10*3.14/360, 0]), useFixedBase=1)

pybullet.resetDebugVisualizerCamera(1.5, 90.0, 0.0, [0.0, 0.0, 0.7])
    #pybullet.resetDebugVisualizerCamera(0.7, 90.0, -89.9, [0.5, 0.0, 0.5])

    #lateralFrictionId = pybullet.addUserDebugParameter("lateral friction", 0, 1, 0.5)

th_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joints = [0, 1, 2, 3, 4, 5]
frag = 0
Force_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

for i in range(20 * f_ctrl * 10000):
    t = i / f_ctrl

    if i % f_ctrl == 0:
        joint_states = pybullet.getJointStates(boxId, joints)
        theta = []
        trq = []
        for j in joint_states:
            theta.append(j[0])
            trq.append(j[-1])
        mymod.set_joint_angle(theta)

        if t < 10000:
            mymod.control_joint()

        if t == 10000:
            #mu = 0.2#IROS実験
            mu = 0.4#IROS実験
            pybullet.enableJointForceTorqueSensor(boxId, jointIndex = 6, enableSensor = True)
            pybullet.changeDynamics(boxId, 7, lateralFriction=mu)
            #pybullet.changeDynamics(pancakeId, 0, contactStiffness=10000,contactDamping=10000)
            pybullet.changeDynamics(pancakeId, 7, contactStiffness=10000,contactDamping=10000)

        if t > 10000:               #
            get_Force = pybullet.getJointState(boxId, 6)[2]     #tikarasensa syutoku
            #mu = 0.5
            #pybullet.changeDynamics(boxId, 7, lateralFriction=mu)
            spatula_state = pybullet.getLinkState(boxId, 7, computeLinkVelocity=1)
            spatula_pos = spatula_state[0]
            Force = pybullet.getJointState(boxId, 6)[2] #力センサ(エンドエフェクタ)の値を取得
            mymod.set_force(Force)
            mymod.control_turnover()
        torque = mymod.get_joint_torque()

    pybullet.setJointMotorControlArray(boxId, joints, pybullet.POSITION_CONTROL, forces=[0.0]*6)
    pybullet.setJointMotorControlArray(boxId, joints, pybullet.TORQUE_CONTROL, forces=torque)
    pybullet.stepSimulation()


cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(boxId)
#print(cubePos, cubeOrn)

pybullet.disconnect()
