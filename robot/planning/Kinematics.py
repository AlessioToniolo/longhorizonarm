import numpy as np
import time
from ..hardware.ServoController import ServoController

class Kinematics:
    def __init__(self, controller):
        self.controller = controller

        self.l1 = 145.1 #mm
        self.l2 = 186.9 #mm
        self.l3 = 162 #mm

    def find_final_angles(self, x, y, z):
        kinematics = self.inverse_kinematics(x, y, z)
        current_angles = self.controller.get_current_angles()
        final_angles = []
        for angle in kinematics:
            final_angles.append(angle)
        final_angles.append(current_angles[4])
        final_angles.append(current_angles[5])

    def move_to_pos_profiled(self, x, y, z):
        final_angles = self.find_final_angles(x,y,z)
        self.controller.move_to_angles_with_profile(final_angles)

    def move_to_pos_synced(self, x, y, z):
        final_angles = self.find_final_angles(x,y,z)
        current_angles = self.controller.get_current_angles()
        diff_angles = final_angles - current_angles

        move_times = []
        for i in range(len(final_angles)):
            move_times.append(self.controller.servos[i].calculate_profile_total_time(abs(diff_angles[i])))
        max_time = max(move_times)

        for servo in self.controller.servos:
            original_time = servo.calculate_profile_total_time(abs(diff_angles[i]))
            scale = original_time / max_time
            servo.max_velocity *= scale
            servo.max_acceleration *= scale**2
        
        self.controller.move_to_angles_with_profile(final_angles)
        time.sleep()
        self.controller.reset_profiles()


    def inverse_kinematics(self, x, y, z, end_effector_angle=0):
        swivel = np.atan2(y, x)
        
        r = np.sqrt(x*x + y*y)

        wx = r - self.l3*np.cos(end_effector_angle)
        wz = z - self.l3*np.sin(end_effector_angle)

        d = np.sqrt(wx**2 + wz**2)
        cos_theta2 = (d*d - self.l1*self.l1 - self.l2*self.l2)/(2*self.l1*self.l2)
        theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))

        theta1 = np.arctan2(wz, wx) - np.arctan2(self.l2*np.sin(theta2), self.l1 + self.l2*np.cos(theta2))

        theta3 = end_effector_angle - theta1 - theta2

        return [swivel, theta1, theta2, theta3]