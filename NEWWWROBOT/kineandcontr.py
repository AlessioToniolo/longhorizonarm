import time
import serial
import serial.tools.list_ports
import json
import numpy as np

class ArduinoServoController:
    def __init__(self, baud_rate=115200):
        self.port = self._find_arduino_port()
        if not self.port:
            raise serial.SerialException("Arduino not found")
        
        self.serial = serial.Serial(
            port=self.port,
            baudrate=baud_rate,
            timeout=1,
            write_timeout=1
        )
        time.sleep(2)  # Wait for Arduino reset
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
    
    def _find_arduino_port(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if "CH340" in port.description or "Arduino" in port.description:
                return port.device
        return ports[0].device if ports else None
    
    def send_command(self, message):
        """Send a command to the Arduino and get response"""
        try:
            cmd = f"{json.dumps(message)}\n"
            self.serial.write(cmd.encode('ascii'))
        except serial.SerialTimeoutException:
            return {"status": "error", "message": "Write timeout"}
        
        try:
            response = self.serial.readline()
            if response:
                return json.loads(response.decode('ascii'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return {"status": "error", "message": "Invalid response"}
        
        return {"status": "error", "message": "No response"}
    
    def move_servo(self, servo_id, target_position):
        """Move a single servo to target position"""
        message = {
            "command": "move",
            "servo_id": servo_id,
            "target": target_position
        }
        return self.send_command(message)
    
    def move_multiple_servos(self, movements):
        """Move multiple servos to target positions"""
        message = {
            "command": "move_multiple",
            "movements": movements
        }
        return self.send_command(message)
    
    def check_status(self):
        message = {"command": "status"}
        return self.send_command(message)
    
    def close(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
class TwoDServoArm:
    def __init__(self, controller, l1=145.1, l2=186.9):
        self.controller = controller
        self.l1 = l1  # Length of the first link in mm
        self.l2 = l2  # Length of the second link in mm
        
        # Set initial position to be upright: x = -l2, y = l1
        self.current_x = -self.l2
        self.current_y = self.l1
        self.update_angles(self.current_x, self.current_y)
    
    def inverse_kinematics(self, x, y):
        """Calculate joint angles for given x, y coordinates"""
        # Distance from origin to target
        d = np.sqrt(x**2 + y**2)
        
        # Check if the point is reachable
        if d > (self.l1 + self.l2) or d < abs(self.l1 - self.l2):
            raise ValueError("Target position is out of reach")
        
        # Calculate elbow angle using the Law of Cosines
        cos_theta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))
        
        # Calculate shoulder angle
        k1 = self.l1 + self.l2 * np.cos(theta2)
        k2 = self.l2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        # Convert to degrees
        theta1_deg = np.degrees(theta1)
        theta2_deg = np.degrees(theta2)
        
        return [theta1_deg, theta2_deg]
    
    def update_angles(self, x, y):
        """Calculate and send new angles to the servos"""
        try:
            angles = self.inverse_kinematics(x, y)
            shoulder_angle, elbow_angle = angles
            
            # Optionally, add offsets if your physical setup requires it
            # For example, if servo 0 needs to be offset by +0 degrees and servo 1 by +90 degrees:
            # shoulder_angle += 0
            # elbow_angle += 90
            
            movements = [
                {
                    "servo_id": 0,  # Shoulder servo
                    "target": shoulder_angle,
                },
                {
                    "servo_id": 1,  # Elbow servo
                    "target": elbow_angle,
                }
            ]
            response = self.controller.move_multiple_servos(movements)
            if response.get("status") == "success":
                self.current_x = x
                self.current_y = y
                print(f"Moved to position x={x:.2f} mm, y={y:.2f} mm")
            else:
                print(f"Error moving servos: {response.get('message')}")
        except ValueError as ve:
            print(f"Error: {ve}")
    
    def move_relative(self, dx, dy):
        """Move the arm by a relative offset"""
        new_x = self.current_x + dx
        new_y = self.current_y + dy
        self.update_angles(new_x, new_y)
    
    def get_current_position(self):
        return (self.current_x, self.current_y)

def main():
    controller = None
    arm = None
    try:
        controller = ArduinoServoController()
        arm = TwoDServoArm(controller)
        print(f"Connected to Arduino on port: {controller.port}")
        print("Initial Position: x = {:.2f} mm, y = {:.2f} mm".format(*arm.get_current_position()))
        
        print("\nCommands:")
        print("  move up <distance>")
        print("  move down <distance>")
        print("  move left <distance>")
        print("  move right <distance>")
        print("  move up <distance> right <distance>")
        print("  move down <distance> left <distance>")
        print("  status - Check status of all servos")
        print("  q - Quit")
        
        while True:
            user_input = input("> ").strip().lower()
            
            if not user_input:
                continue
                
            if user_input == 'q':
                break
            elif user_input == 'status':
                status = controller.check_status()
                print(f"Status: {json.dumps(status, indent=2)}")
            elif user_input.startswith('move'):
                try:
                    parts = user_input.split()
                    if len(parts) < 3:
                        print("Invalid move command. Example: move up 10 right 20")
                        continue
                    
                    # Parse commands
                    dx, dy = 0.0, 0.0
                    i = 1
                    while i < len(parts):
                        direction = parts[i]
                        distance = float(parts[i+1])
                        if direction == 'up':
                            dy += distance
                        elif direction == 'down':
                            dy -= distance
                        elif direction == 'left':
                            dx -= distance
                        elif direction == 'right':
                            dx += distance
                        else:
                            print(f"Unknown direction: {direction}")
                            break
                        i += 2
                    else:
                        # Apply movement
                        arm.move_relative(dx, dy)
                        continue  # Skip the error message
                    print("Invalid move command format.")
                except (ValueError, IndexError):
                    print("Invalid move command. Example: move up 10 right 20")
            else:
                print("Unknown command. Available commands: move, status, q")
                    
    except KeyboardInterrupt:
        print("\nExiting...")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        if arm:
            arm.controller.close()

if __name__ == "__main__":
    main()
