import numpy as np
from ArduinoServoController import ArduinoServoController
import time

class TestKinematics:
    def __init__(self):
        self.controller = ArduinoServoController()
        # Servo IDs
        self.base_servo = 0     # Base rotation
        self.shoulder_servo = 1  # First joint
        self.elbow_servo = 2    # Second joint
        
        # Link lengths in mm
        self.l1 = 145.1  # First link length
        self.l2 = 186.9  # Second link length
        
    def inverse_kinematics(self, x, y, z):
        """Calculate joint angles for given x,y,z coordinate"""
        # Base angle (rotation around z-axis)
        theta1 = np.degrees(np.arctan2(y, x))
        
        # Distance from base to target in x-y plane
        r = np.sqrt(x*x + y*y)
        
        # Solve for shoulder and elbow joints using 2D kinematics
        d = np.sqrt(r*r + z*z)
        
        # Check if point is reachable
        if d > (self.l1 + self.l2) or d < abs(self.l1 - self.l2):
            raise ValueError("Position out of reach")
        
        # Law of cosines for elbow angle
        cos_theta3 = (d*d - self.l1*self.l1 - self.l2*self.l2)/(2*self.l1*self.l2)
        theta3 = np.degrees(np.arccos(np.clip(cos_theta3, -1.0, 1.0)))
        
        # Shoulder angle
        theta2 = np.degrees(np.arctan2(z, r) - np.arctan2(self.l2*np.sin(np.radians(theta3)), 
                                                         self.l1 + self.l2*np.cos(np.radians(theta3))))
        
        return [theta1, theta2, theta3]
    
    def move_to_position(self, x, y, z):
        """Move arm to specified position"""
        try:
            angles = self.inverse_kinematics(x, y, z)
            message = {
                "command": "move_multiple",
                "movements": [
                    {"servo_id": self.base_servo, "position": angles[0]},
                    {"servo_id": self.shoulder_servo, "position": angles[1]},
                    {"servo_id": self.elbow_servo, "position": angles[2]}
                ]
            }
            return self.controller.send_command(message)
        except ValueError as e:
            print(f"Error: {e}")
            return {"status": "error", "message": str(e)}
    
    def test_movements(self):
        """Test various positions"""
        print("Starting kinematics test...")
        
        # Test positions (x, y, z in mm)
        positions = [
            (200, 0, 100),    # Front
            (141, 141, 100),  # Front-right
            (0, 200, 100),    # Right
            (200, 0, 150),    # Front-raised
        ]
        
        for x, y, z in positions:
            print(f"\nMoving to position: x={x}, y={y}, z={z}")
            result = self.move_to_position(x, y, z)
            print(f"Result: {result}")
            time.sleep(2)  # Wait for movement to complete
            
        print("\nTest complete")
    
    def close(self):
        self.controller.close()

def main():
    arm = TestKinematics()
    try:
        arm.test_movements()
    finally:
        arm.close()

if __name__ == "__main__":
    main()