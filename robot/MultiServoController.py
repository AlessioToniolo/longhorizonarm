import time
import serial
import serial.tools.list_ports
import json

class ArduinoProfiledServoController:
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
    
    def move_with_profile(self, servo_id, target_position, max_vel=45, max_accel=15):
        """Send a profiled move command to a single servo"""
        message = {
            "command": "move_profiled",
            "servo_id": servo_id,
            "target": target_position,
            "max_vel": max_vel,
            "max_accel": max_accel
        }
        
        return self.send_command(message)
    
    def move_multiple_with_profile(self, movements):
        """Send multiple profiled move commands to the Arduino"""
        message = {
            "command": "move_multiple_profiled",
            "movements": movements
        }
        
        return self.send_command(message)
    
    def check_status(self):
        message = {"command": "status"}
        return self.send_command(message)
    
    def close(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

def main():
    controller = None
    try:
        controller = ArduinoProfiledServoController()
        print(f"Connected to Arduino on port: {controller.port}")
        
        print("Commands:")
        print("  move <id> <pos> [max_vel] [max_accel] - Move a single servo with profile")
        print("  move_multiple <id1> <pos1> [max_vel1] [max_accel1] ; <id2> <pos2> [max_vel2] [max_accel2] ; ...")
        print("  status - Check status of all servos")
        print("  q - Quit")
        
        while True:
            user_input = input("> ").strip()
            
            if not user_input:
                continue
                
            if user_input.lower() == 'q':
                break
            elif user_input.lower() == 'status':
                status = controller.check_status()
                print(f"Status: {json.dumps(status, indent=2)}")
            elif user_input.startswith('move_multiple'):
                # Example input:
                # move_multiple 0 90 60 30 ; 1 120 50 25
                try:
                    _, movements_str = user_input.split(' ', 1)
                    movements = []
                    for move_str in movements_str.split(';'):
                        parts = move_str.strip().split()
                        if len(parts) < 2:
                            print("Invalid movement format. Usage: move_multiple <id1> <pos1> [max_vel1] [max_accel1] ; ...")
                            break
                        servo_id = int(parts[0])
                        pos = float(parts[1])
                        max_vel = float(parts[2]) if len(parts) > 2 else 60
                        max_accel = float(parts[3]) if len(parts) > 3 else 30
                        if 0 <= servo_id < 6 and 0 <= pos <= 270:
                            movements.append({
                                "servo_id": servo_id,
                                "target": pos,
                                "max_vel": max_vel,
                                "max_accel": max_accel
                            })
                        else:
                            print("Servo ID must be between 0 and 5, and position must be between 0 and 270 degrees")
                            break
                    else:
                        print(f"Moving multiple servos: {movements}")
                        response = controller.move_multiple_with_profile(movements)
                        print(f"Response: {response}")
                except ValueError:
                    print("Invalid input format. Usage: move_multiple <id1> <pos1> [max_vel1] [max_accel1] ; ...")
            elif user_input.startswith('move'):
                # Example input:
                # move 0 90 60 30
                try:
                    parts = user_input.split()
                    if len(parts) >= 3:
                        servo_id = int(parts[1])
                        position = float(parts[2])
                        max_vel = float(parts[3]) if len(parts) > 3 else 60
                        max_accel = float(parts[4]) if len(parts) > 4 else 30
                        
                        if 0 <= servo_id < 6 and 0 <= position <= 270:
                            print(f"Moving servo {servo_id} to position: {position} degrees with max_vel={max_vel}, max_accel={max_accel}")
                            response = controller.move_with_profile(servo_id, position, max_vel, max_accel)
                            print(f"Response: {response}")
                        else:
                            print("Servo ID must be between 0 and 5, and position must be between 0 and 270 degrees")
                    else:
                        print("Usage: move <id> <pos> [max_vel] [max_accel]")
                except (ValueError, IndexError):
                    print("Invalid input format. Usage: move <id> <pos> [max_vel] [max_accel]")
            else:
                print("Unknown command. Available commands: move, move_multiple, status, q")
                    
    except KeyboardInterrupt:
        print("\nExiting...")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        if controller:
            controller.close()

if __name__ == "__main__":
    main()
