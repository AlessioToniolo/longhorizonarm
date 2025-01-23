import numpy as np
from ..hardware.ServoController import ServoController

def inverse_kinematics(x, y, z, l1, l2, l3):
    swivel = np.atan2(y, x)
    
    r = np.sqrt(x*x + y*y)

    theta1 = np.atan2(z, r)

    d = np.sqrt(r*r + z*z)

    if d > (l2 + l3) or d < abs(l2 - l3):
        raise Exception("out of reach")
    
    cos_theta3 = (d*d - l2*l2 - l3*l3)/(2*l2*l3)

    if cos_theta3 > 1 or cos_theta3 < -1:
        raise Exception("out of reach")
    
    theta3 = np.arccos(cos_theta3)
    
    beta = np.arctan2(z, r)
    gamma = np.arccos((l2*l2 + d*d - l3*l3)/(2*l2*d))
    theta2 = beta + gamma

    return [theta1, theta2, theta3]