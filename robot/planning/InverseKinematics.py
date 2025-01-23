import numpy as np
from ..hardware.ServoController import ServoController

def inverse_kinematics(x, y, z, l1, l2, l3, end_effector_angle=0):
    swivel = np.atan2(y, x)
    
    r = np.sqrt(x*x + y*y)

    wx = r - l3*np.cos(end_effector_angle)
    wz = z - l3*np.sin(end_effector_angle)

    d = np.sqrt(wx**2 + wz**2)
    cos_theta2 = (d*d - l1*l1 - l2*l2)/(2*l1*l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))

    theta1 = np.arctan2(wz, wx) - np.arctan2(l2*np.sin(theta2), l1 + l2*np.cos(theta2))

    theta3 = end_effector_angle - theta1 - theta2

    return [swivel, theta1, theta2, theta3]