import numpy as np
from scipy.spatial.transform import Rotation as R
import math
# import scipy.spatial.transform

# This class provides an interface to initialize from and represent rotations with:
# Quaternions
# Rotation Matrices
# Rotation Vectors
# Euler Angles

# r = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
# r = R.from_dcm([
#     [0, -1, 0],
#     [1, 0, 0],
#     [0, 0, 1]
# ])

# x y z
# raw: 0.314781486988, yaw: 0.186363324523, roll: 1.18304109573
# pitch: 0.314781486988, yaw: 0.186363324523, roll: 1.18304109573

# pitch: 0.648039400578, yaw: 0.299980163574, roll: 0.313891947269

# euler zyx or ?
def toRadians(dgr):
    return math.radians(dgr / math.pi)

def toDegree(rad):
    return math.degrees(rad)


print(toDegree(0.648039400578))
print(toDegree(0.299980163574))
print(toDegree(0.313891947269))


r = R.from_euler('zyx', [1.18304109573, 0.314781486988, 0.186363324523], degrees=False)
# r = R.from_euler('zyx', [[90, 45, 30]], degrees=True)
print(r.as_rotvec())


#        [[0.34924802 0.16745691 1.19857639]]
# robot: [0.00808362542162, -3.11507649905, 0.0115797277352]
0.007933661237571376, -3.114925314561998, 0.011061973276916085

r = R.from_rotvec([0.00808362542162, -3.11507649905, 0.0115797277352])
print(r.as_euler('zxy', degrees=True))

r = R.from_euler('zxy',[-72.42124581, 1.57620936, -179.93848198], degrees=True)
print(r.as_rotvec())

# print( math.radians(180 / math.pi))
# math.radians(180)



# print(r.as_euler('zyx', degrees=True))