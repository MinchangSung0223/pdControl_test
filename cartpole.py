import pybullet as p
import time
import pybullet_data
import numpy as np
import math
useMaximalCoordinates = False
p.connect(p.GUI)
p.setGravity(0, 0, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
pole = p.loadURDF("cartpole.urdf", [0, 0, 0], useMaximalCoordinates=useMaximalCoordinates)

for i in range(p.getNumJoints(pole)):
  p.setJointMotorControl2(pole, i, p.POSITION_CONTROL, targetPosition=0, force=0)

timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)


numJoints = p.getNumJoints(pole)

desired_q = np.array([2,0]);
desired_qdot = np.array([0,0]);
kpCart = 500
kdCart = 150
kpPole = 10
kdPole = 1
prev_q = 0

kps = [kpCart, kpPole]
kds = [kdCart, kdPole]
while p.isConnected():
	timeStep = p.readUserDebugParameter(timeStepId)
	p.setTimeStep(timeStep)
	jointStates = p.getJointStates(pole,[0,1])
	q = np.array([jointStates[0][0],jointStates[1][0]])
	qdot = np.array((q-prev_q )/timeStep)

	qError = desired_q - q;
	qdotError = desired_qdot - qdot;
	Kp = np.diagflat(kps)
	Kd = np.diagflat(kds)
	forces = Kp.dot(qError)+Kd.dot(qdotError)
	maxF = 1000
	force1 = np.clip(forces[0],-maxF,maxF)
	maxF = 1000
	force2 = np.clip(forces[1],-maxF,maxF)
	forces = [force1,force2]
	print(forces)
	p.setJointMotorControlArray(pole, [0,1], controlMode=p.TORQUE_CONTROL, forces=forces)
	prev_q = q
	#print("error : ", qError, ", forces : ",forces)
	
	
	if (not useRealTimeSim):
		p.stepSimulation()
		time.sleep(timeStep)
