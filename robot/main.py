from robot.hardware.ServoController import ServoController
from robot.hardware.Servo import Servo
from robot.planning.Kinematics import Kinematics
import numpy as np
import time

def test_servo_init():
    controller = ServoController()
    angles = controller.get_current_angles()
    controller.home()
    home_angles = controller.get_current_angles()
    controller.stop()
    return home_angles == controller.home_positions

def test_basic_movement():
    controller = ServoController()
    test_angles = [45, 90, 135, 90, 90, 90]
    controller.move_to_angles(test_angles)
    current_angles = controller.get_current_angles()
    controller.stop()
    return np.allclose(current_angles, test_angles)

def test_gripper():
    controller = ServoController()
    controller.change_gripper(0)
    time.sleep(1)
    controller.change_gripper(100)
    time.sleep(1)
    controller.stop()
    return True

def test_motion_profiles():
    controller = ServoController()
    start_angles = [90, 90, 90, 90, 90, 90]
    end_angles = [180, 180, 180, 180, 180, 180]
    controller.move_to_angles(start_angles)
    time.sleep(2)
    controller.move_to_angles_with_profile(end_angles)
    controller.stop()
    return True

def test_kinematics():
    controller = ServoController()
    kinematics = Kinematics(controller)
    x, y, z = 150, 0, 175
    angles = kinematics.inverse_kinematics(x, y, z)
    controller.stop()
    return len(angles) == 4

def test_workspace_movement():
    controller = ServoController()
    kinematics = Kinematics(controller)
    positions = [
        (200, 0, 200),
        (200, 200, 200),
        (200, -200, 200),
        (200, 0, 0)
    ]
    for x, y, z in positions:
        kinematics.move_to_pos_profiled(x, y, z)
        time.sleep(2)
    controller.stop()
    return True

def test_basic_path_finder():
    controller = ServoController()
    kinematics = Kinematics(controller)
    path_finder = PathFinder(kinematics)

    start = np.array([100, 0, 100])
    end = np.array([200, 200, 100])
    obstacle = np.array([150, 100, 100])  # obstacle center
    obstacle_size = 50  # mm

    path = path_finder.find_path(start, end, obstacle, obstacle_size)
    path_finder.execute_path(path)

def main():
    # test_servo_init()
    # test_basic_movement()
    # test_gripper()
    # test_motion_profiles()
    # test_kinematics()
    # test_workspace_movement()
    # test_basic_path_finder()
    pass

if __name__ == "__main__":
    main()