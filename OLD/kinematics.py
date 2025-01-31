import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import serial
import json
import time
from typing import Optional, List, Tuple

class RobotArm:
    def __init__(self, port: str = 'COM5', baudrate: int = 115200):
        """
        Initialize robot arm with IK chain and serial connection
        
        Physical dimensions (in mm):
        - Base height to first joint: 145.1
        - First arm segment: 186.9
        - Second arm segment: 162.0
        - Third arm segment (to end effector): 150.0
        """
        # Physical dimensions in meters
        self.base_height = 0.1451    # 145.1mm
        self.link1_length = 0.1869   # 186.9mm
        self.link2_length = 0.1620   # 162.0mm
        self.link3_length = 0.1500   # 150.0mm (provisional)
        
        # Create kinematic chain
        self.chain = Chain(
            name='arm',
            active_links_mask=[False, True, True, True, True, False],  # Explicitly mark fixed links as inactive
            links=[
                OriginLink(),  # Base (fixed)
                URDFLink(      # Base rotation (Servo 0, Pin 13)
                    name="base_rotation",
                    origin_translation=[0, 0, self.base_height],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],  # Z-axis rotation
                    bounds=(-np.pi/2, np.pi/2)  # -90 to 90 degrees
                ),
                URDFLink(      # Shoulder 1 (Servo 1, Pin 12)
                    name="shoulder",
                    origin_translation=[0, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # Y-axis rotation
                    bounds=(-np.pi/2, np.pi/2)
                ),
                URDFLink(      # Shoulder 2 (Servo 2, Pin 14)
                    name="elbow",
                    origin_translation=[self.link1_length, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # Y-axis rotation
                    bounds=(-np.pi/2, np.pi/2)
                ),
                URDFLink(      # Shoulder 3 (Servo 3, Pin 27)
                    name="wrist",
                    origin_translation=[self.link2_length, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # Y-axis rotation
                    bounds=(-np.pi/2, np.pi/2)
                ),
                URDFLink(      # End effector (fixed)
                    name="end_effector",
                    origin_translation=[self.link3_length, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 0],
                    bounds=(0, 0)  # Fixed link
                )
            ]
        )
        
        # Setup serial connection
        self.serial = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Allow time for Arduino to reset
        
    def solve_ik(self, x: float, y: float, z: float, orientation: float = 0) -> Optional[List[float]]:
        """
        Solve inverse kinematics for target position
        
        Args:
            x, y, z: Target position in millimeters
            orientation: End effector orientation in degrees (0 is horizontal)
            
        Returns:
            List of 4 joint angles in degrees, or None if no solution
        """
        try:
            # Convert to meters
            x, y, z = x/1000, y/1000, z/1000
            
            # Create target vector (position only)
            target_position = np.array([x, y, z])
            
            # Calculate orientation vector based on desired angle
            if orientation:
                orientation_rad = np.radians(orientation)
                orientation_vector = np.array([
                    np.cos(orientation_rad),
                    0,
                    np.sin(orientation_rad)
                ])
            else:
                orientation_vector = np.array([1, 0, 0])  # Default horizontal orientation
            
            # Get initial position (all joints at 0)
            initial_position = np.zeros(len(self.chain.links))
            
            # Solve IK using position and orientation
            print(f"Attempting to solve IK for position: ({x}, {y}, {z})")
            angles = self.chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=orientation_vector,
                initial_position=initial_position
            )
            
            # Convert to degrees and offset by 90 (servo center position)
            # Skip first (base) and last (end effector) links
            degrees = np.degrees(angles[1:-1])
            degrees += 90
            
            # Check solution is within servo range (0-180 degrees)
            if not all(0 <= angle <= 180 for angle in degrees):
                print("Solution outside servo range")
                return None
            
            print(f"Found solution (degrees): {degrees.tolist()}")
            return degrees.tolist()
            
        except Exception as e:
            print(f"IK solver error: {e}")
            return None
            
    def move_servo(self, servo_id: int, position: float, use_profile: bool = False) -> None:
        """
        Move a single servo to specified position
        
        Args:
            servo_id: Servo number (0-3)
            position: Target angle in degrees (0-180)
            use_profile: Whether to use motion profiling
        """
        command = {
            "servo": servo_id,
            "position": position,
            "profile": use_profile
        }
        self.serial.write(json.dumps(command).encode() + b'\n')
        time.sleep(0.1)  # Small delay between commands
        
    def move_to(self, x: float, y: float, z: float, 
                orientation: float = 0, use_profile: bool = False) -> bool:
        """
        Move end effector to target position
        
        Args:
            x, y, z: Target position in millimeters
            orientation: End effector orientation in degrees
            use_profile: Whether to use motion profiling
            
        Returns:
            True if movement was initiated, False if no solution found
        """
        angles = self.solve_ik(x, y, z, orientation)
        if angles is None:
            return False
            
        for i, angle in enumerate(angles):
            self.move_servo(i, angle, use_profile)
        return True
    
    def center_all(self, use_profile: bool = False) -> None:
        """Center all servos (move to 90 degrees)"""
        for servo_id in range(4):
            self.move_servo(servo_id, 90, use_profile)
    
    def close(self):
        """Close serial connection"""
        self.serial.close()

def main():
    # Test the robot arm
    arm = RobotArm()  # Adjust COM port if needed
    
    try:
        print("Initializing arm position...")
        # Center all servos and wait for completion
        arm.center_all(use_profile=False)
        print("Waiting for arm to center (5 seconds)...")
        time.sleep(5)  # Long delay to ensure arm reaches position
        
        print("\nTesting arm movements...")
        
        # Test coordinates (in mm)
        test_positions = [
            #(200, 0, 300),     # Forward and up
            (200, 200, 300),   # Forward-right and up
            #(0, 200, 300),     # Right side
        ]
        
        for x, y, z in test_positions:
            print(f"\nMoving to position: ({x}, {y}, {z})mm")
            if arm.move_to(x, y, z):
                print("Movement initiated")
                time.sleep(3)  # Wait for movement to complete
            else:
                print("No solution found")
        
        print("\nReturning to center position...")
        arm.center_all()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        arm.close()

if __name__ == "__main__":
    main()