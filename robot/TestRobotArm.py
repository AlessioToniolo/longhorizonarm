from ArduinoServoController import ArduinoServoController
import time

class TestRobotArm:
    def __init__(self):
        self.controller = ArduinoServoController()
        self.pivot_one_servo = 0    # Pivot one rotation servo ID
        self.pivot_two_servo = 1    # Pivot two servo ID
        
    def move_servos(self, pivot_one_angle, pivot_two_angle):
        """Move multiple servos simultaneously"""
        message = {
            "command": "move_multiple",
            "movements": [
                {"servo_id": self.pivot_one_servo, "position": pivot_one_angle},
                {"servo_id": self.pivot_two_servo, "position": pivot_two_angle}
            ]
        }
        return self.controller.send_command(message)
        
    def test_movement(self):
        """Test basic movements"""
        print("Testing servo movements...")
        
        # Test pivot movements
        self.move_servos(90, 90)  # Center position
        time.sleep(1)
        self.move_servos(45, 45)  # Left position
        time.sleep(1)
        self.move_servos(135, 135) # Right position
        time.sleep(1)
        
        print("Test complete")
        
    def close(self):
        self.controller.close()

def main():
    arm = TestRobotArm()
    try:
        arm.test_movement()
    finally:
        arm.close()

if __name__ == "__main__":
    main()