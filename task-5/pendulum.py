# Задача 5. Траектория для маятника
# обеспечить переход двухзвенного маятника в заданное декартово положение за счет вычисления необходимого положения джоинтов
# через p.calculateInverseKinematics в режиме POSITION_CONTROL




import pybullet as p
import time
import pybullet_data
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import os

guiFlag = True

dt = 1/240 # pybullet simulation step
th0 = 0.5  # starting position (radian)
thd = 1.0  # desired position (radian)
kp = 40.0  # proportional coefficient
ki = 40.0
kd = 20.0
g = 10     # m/s^2
L = 0.8    # m
L1 = L
L2 = L
m = 1      # kg
f0 = 10    # applied const force

xd = 0.5
zd = 1

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
# planeId = p.loadURDF("plane.urdf")
URDF_PATH = os.path.join(os.path.dirname(__file__), "two-link.urdf.xml")
boxId = p.loadURDF(URDF_PATH, useFixedBase=True)

# get rid of all the default damping forces
# think of it as imagined "air drag"
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

pos0 = p.getLinkState(boxId, 4)[0]
X0 = np.array([[pos0[0]],[pos0[2]]])

maxTime = 5 # seconds
logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
logXsim = np.zeros(sz)
logZsim = np.zeros(sz)
idx = 0
T = 2
for t in logTime:

    pos = p.getLinkState(boxId, 4)[0] # текущая позиция
    logXsim[idx] = pos[0]
    logZsim[idx] = pos[2]

    Xd = np.array([[xd],[zd]]) # цель, финальная точка

    s = 1 
    if t < T:
        s = (3/T**2) * t**2 -2/(T**3) * t**3
    Xd_curr = X0 + s * (Xd - X0) 

    targetPos = [
        float(Xd_curr[0]),
        0,
        float(Xd_curr[1])
    ]

    jointPoses = p.calculateInverseKinematics( # вычисляем обратную кинематику
        bodyUniqueId=boxId, # идентификатор тела
        endEffectorLinkIndex=4, # индекс звена
        targetPosition=targetPos, # целевая точка, куда должен попасть конец манипулятора
    )


    p.setJointMotorControlArray(
        bodyIndex=boxId,
        jointIndices=[1,3],
        targetPositions=[jointPoses[0],jointPoses[1]],
        controlMode=p.POSITION_CONTROL # режим управления по положению
    )
    p.stepSimulation()

    idx += 1
    if guiFlag:
        time.sleep(dt)
p.disconnect()

plt.subplot(2,1,1)
plt.plot(logTime, logXsim)
plt.subplot(2,1,2)
plt.plot(logTime, logZsim)
plt.show()