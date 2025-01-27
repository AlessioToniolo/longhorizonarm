import numpy as np
from MultiServoController import ArduinoProfiledServoController
import time
import json

class MultiServoKinematics:
    def __init__(self, default_max_vel_shoulder=30, default_max_accel_shoulder=10,
                 default_max_vel_elbow=15, default_max_accel_elbow=5):
        self.controller = ArduinoProfiledServoController()
        # Servo IDs
        self.shoulder_servo = 0  # First joint
        self.elbow_servo = 1    # Second joint
        
        # Link lengths in mm
        self.l1 = 145.1  # First link length
        self.l2 = 186.9  # Second link length
        
        # Default motion profiles
        self.default_max_vel_shoulder = default_max_vel_shoulder
        self.default_max_accel_shoulder = default_max_accel_shoulder
        self.default_max_vel_elbow = default_max_vel_elbow
        self.default_max_accel_elbow = default_max_accel_elbow

        # Initialize starting position (fully upright)
        self.current_x = 0
        self.current_y = self.l1 + self.l2  # Full height
        self.current_z = 0

    def inverse_kinematics(self, x, y, z):
        """Calculate joint angles for given x,y,z coordinate"""
        # Distance from base to target in x-y plane
        distance = np.sqrt(x**2 + y**2)
        
        # Check reachability
        if distance > (self.l1 + self.l2):
            raise ValueError("Target position is out of reach.")
        
        # Law of cosines for elbow angle
        cos_angle2 = (distance**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_angle2 = np.clip(cos_angle2, -1, 1)  # Prevent domain errors
        theta2 = np.degrees(np.arccos(cos_angle2))
        
        # Shoulder angle
        beta = np.arctan2(y, x)
        alpha = np.arccos((distance**2 + self.l1**2 - self.l2**2)/(2*self.l1*distance))
        theta1 = np.degrees(beta + alpha)
        
        return [theta1, theta2]
    
    def move_to_position(self, x, y, z, 
                         max_vel_shoulder=None, max_accel_shoulder=None,
                         max_vel_elbow=None, max_accel_elbow=None):
        """Move the arm to the specified (x, y, z) position with optional velocity and acceleration."""
        angles = self.inverse_kinematics(x, y, z)
        
        # Use default values if not specified
        vel_shoulder = max_vel_shoulder if max_vel_shoulder is not None else self.default_max_vel_shoulder
        accel_shoulder = max_accel_shoulder if max_accel_shoulder is not None else self.default_max_accel_shoulder
        vel_elbow = max_vel_elbow if max_vel_elbow is not None else self.default_max_vel_elbow
        accel_elbow = max_accel_elbow if max_accel_elbow is not None else self.default_max_accel_elbow
        
        movements = [
            {
                "servo_id": self.shoulder_servo,
                "target": angles[0],
                "max_vel": vel_shoulder,
                "max_accel": accel_shoulder
            },
            {
                "servo_id": self.elbow_servo,
                "target": angles[1],
                "max_vel": vel_elbow,
                "max_accel": accel_elbow
            }
        ]
        
        return self.controller.move_multiple_with_profile(movements)
    
    def move_left(self, distance, 
                  max_vel_shoulder=None, max_accel_shoulder=None,
                  max_vel_elbow=None, max_accel_elbow=None):
        """Move the arm left by the specified distance."""
        return self.move_relative(-distance, 0, 0,
                                 max_vel_shoulder, max_accel_shoulder,
                                 max_vel_elbow, max_accel_elbow)

    def move_right(self, distance, 
                   max_vel_shoulder=None, max_accel_shoulder=None,
                   max_vel_elbow=None, max_accel_elbow=None):
        """Move the arm right by the specified distance."""
        return self.move_relative(distance, 0, 0,
                                 max_vel_shoulder, max_accel_shoulder,
                                 max_vel_elbow, max_accel_elbow)

    def move_up(self, distance, 
                max_vel_shoulder=None, max_accel_shoulder=None,
                max_vel_elbow=None, max_accel_elbow=None):
        """Move the arm up by the specified distance."""
        return self.move_relative(0, distance, 0,
                                 max_vel_shoulder, max_accel_shoulder,
                                 max_vel_elbow, max_accel_elbow)

    def move_down(self, distance, 
                  max_vel_shoulder=None, max_accel_shoulder=None,
                  max_vel_elbow=None, max_accel_elbow=None):
        """Move the arm down by the specified distance."""
        return self.move_relative(0, -distance, 0,
                                 max_vel_shoulder, max_accel_shoulder,
                                 max_vel_elbow, max_accel_elbow)

    def move_relative(self, dx, dy, dz, 
                      max_vel_shoulder=None, max_accel_shoulder=None,
                      max_vel_elbow=None, max_accel_elbow=None):
        """Move relative to current position with constraints"""
        # Calculate new position
        new_x = self.current_x + dx
        new_y = self.current_y + dy
        new_z = self.current_z + dz
        
        # Check if position is reachable
        distance = np.sqrt(new_x**2 + new_y**2)
        if distance > (self.l1 + self.l2):
            print("Target position out of reach. Movement aborted.")
            return False
            
        # Prevent upward movement beyond starting height
        if new_y > (self.l1 + self.l2):
            print("Cannot move up beyond starting height. Movement aborted.")
            return False
        
        # Update position
        self.current_x = new_x
        self.current_y = new_y
        self.current_z = new_z
        
        try:
            return self.move_to_position(new_x, new_y, new_z,
                                        max_vel_shoulder, max_accel_shoulder,
                                        max_vel_elbow, max_accel_elbow)
        except ValueError as e:
            print(e)
            return False

    def get_current_position(self):
        """Return the current (x, y, z) position."""
        return self.current_x, self.current_y, self.current_z

    def test_movements(self):
        """Test various positions"""
        print("Starting kinematics test (base rotation disabled)...")
        
        # Test positions (x, y, z in mm)
        # Only testing positions along x-z plane since base rotation is disabled
        positions = [
            (200, 0, 100),    # Front
            (200, 0, 150),    # Front-raised
            (150, 0, 200),    # Up
            (100, 0, 50),     # Low front
        ]
        
        for x, y, z in positions:
            # Note: y coordinate kept at 0 since base rotation is disabled
            print(f"\nMoving to position: x={x}, y=0, z={z}")
            self.move_to_position(x, 0, z)  # Force y to 0
            # Wait until all motions are complete
            while True:
                status = self.controller.check_status()
                moving = any(s.get("status") == "moving" for s in status.get("statuses", []))
                if not moving:
                    break
                time.sleep(0.1)  # Poll every 100ms
            print("Movement complete.")
            time.sleep(1)  # Brief pause before next movement
            
        print("\nTest complete")
    
    def close(self):
        self.controller.close()

def main():
    arm = MultiServoKinematics()
    try:
        print("Connected to servo controller")
        print("Commands:")
        print("  left <distance>   - Move left by distance in mm")
        print("  right <distance>  - Move right by distance in mm")
        print("  up <distance>     - Move up by distance in mm")
        print("  down <distance>   - Move down by distance in mm")
        print("  status            - Check status of all servos")
        print("  q                 - Quit")
        
        while True:
            user_input = input("> ").strip()
            
            if not user_input:
                continue
                
            if user_input.lower() == 'q':
                break
            elif user_input.lower() == 'status':
                status = arm.controller.check_status()
                print(f"Status: {json.dumps(status, indent=2)}")
            else:
                parts = user_input.split()
                command = parts[0].lower()
                if command in ['left', 'right', 'up', 'down']:
                    if len(parts) != 2:
                        print(f"Usage: {command} <distance>")
                        continue
                    try:
                        distance = float(parts[1])
                        if distance < 0:
                            print("Distance must be a positive number.")
                            continue
                        print(f"Executing command: {command} {distance} mm")
                        if command == 'left':
                            success = arm.move_left(distance)
                        elif command == 'right':
                            success = arm.move_right(distance)
                        elif command == 'up':
                            success = arm.move_up(distance)
                        elif command == 'down':
                            success = arm.move_down(distance)
                        
                        if success:
                            # Wait for movement to complete
                            while True:
                                status = arm.controller.check_status()
                                moving = any(s.get("status") == "moving" for s in status.get("statuses", []))
                                if not moving:
                                    break
                                time.sleep(0.1)  # Poll every 100ms
                            print("Movement complete.")
                        else:
                            print("Movement failed.")
                    except ValueError:
                        print("Invalid distance. Please enter a numerical value.")
                else:
                    print("Unknown command. Available commands: left, right, up, down, status, q")
                    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        arm.close()

if __name__ == "__main__":
    main()
