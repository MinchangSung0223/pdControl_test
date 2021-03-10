import pybullet as p
import time
import pybullet_data
import numpy as np
import math


## setup
useMaximalCoordinates = False
p.connect(p.GUI)
p.setGravity(0, 0, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

pole = p.loadURDF("threelink_robot/threelink.urdf", [0, 0, 0], useMaximalCoordinates=useMaximalCoordinates)
p.resetBasePositionAndOrientation(pole, [0, 0, 0], [0, 0, 0, 1])

p.setJointMotorControl2(pole, 0, p.POSITION_CONTROL, targetPosition=0, force=0)
p.setJointMotorControl2(pole, 1, p.POSITION_CONTROL, targetPosition=0, force=0)
p.setJointMotorControl2(pole, 2, p.POSITION_CONTROL, targetPosition=0, force=0)

timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)


numJoints = p.getNumJoints(pole)

desired_q = np.array([0,0,0]);
desired_qdot = np.array([0,0,0]);
prev_q = np.array([0,0,0]);
kps = [500,500,500 ]
kds = [100,100,100 ]
maxF = np.array([100,100,100]);


while p.isConnected():
	timeStep = p.readUserDebugParameter(timeStepId)
	p.setTimeStep(timeStep)
	jointStates = p.getJointStates(pole,[0,1,2])
	q = np.array([jointStates[0][0],jointStates[1][0],jointStates[2][0]])
	qdot = np.array((q-prev_q )/timeStep)


###       CONTROL LOOP
	qError = desired_q - q;
	qdotError = desired_qdot - qdot;
	Kp = np.diagflat(kps)
	Kd = np.diagflat(kds)
	torques = Kp.dot(qError)+Kd.dot(qdotError)
	
	torques = np.clip(torques,-maxF,maxF)
	prev_q = q

###	


	p.setJointMotorControlArray(pole, [0,1,2], controlMode=p.TORQUE_CONTROL, forces=torques)
	
	if (not useRealTimeSim):
		p.stepSimulation()
		time.sleep(timeStep)
