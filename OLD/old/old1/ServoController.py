from .Servo import Servo
import numpy as np

class ServoController:
    def __init__(self):
        self.swivel = Servo(3, 0, 270, 90)
        self.tilt = Servo(5, 0, 270, 90)
        self.second_tilt = Servo(7, 0, 270, 90)
        self.twist = Servo(11, 0, 270, 90) # note, twist comes before wrist on actual embodiment
        self.wrist = Servo(13, 0, 270, 90)
        self.gripper = Servo(15, 0, 270, 90)
        self.servos = [self.swivel, self.tilt, self.second_tilt, self.wrist, self.twist, self.gripper]
        self.home_positions = [servo.get_home() for servo in self.servos]
        self.limits = [[servo.get_min_angle(), servo.get_max_angle()] for servo in self.servos]

    def move_to_angles(self, angles):
        for i in range(len(angles)):
            self.servos[i].set_angle(angles[i])

    def move_to_angles_with_profile(self, angles):
        for i in range(len(angles)):
            self.servos[i].set_angle_with_profile(angles[i])

    def reset_profiles(self):
        for servo in self.servos:
            servo.reset_profile()

    def get_current_angles(self):
        return [servo.get_angle() for servo in self.servos]
    
    def change_gripper(self, percent_open):
        if not 0 <= percent_open <= 100:
            raise Exception("percent_open must be between 0 and 100")
        angle = np.interp(percent_open, [self.gripper.get_min_angle(), self.gripper.get_max_angle()], [0, 180])
        self.gripper.set_angle(angle)

    def home(self):
        self.move_to_angles(self.home_positions)

    def stop(self):
        for servo in self.servos:
            servo.cleanup()
        self.servos = []