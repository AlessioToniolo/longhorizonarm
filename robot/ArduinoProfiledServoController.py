import time
import serial
import serial.tools.list_ports
import json

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
    
    def close(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

class ArduinoProfiledServoController(ArduinoServoController):
    def move_with_profile(self, servo_id, target_position, max_vel=45, max_accel=15):
        """Send profiled move command to Arduino"""
        message = {
            "command": "move_profiled",
            "servo_id": servo_id,
            "target": target_position,
            "max_vel": max_vel,
            "max_accel": max_accel
        }
        
        response = self.send_command(message)
        
        if response.get("status") != "success":
            return response
        
        # Wait until motion is complete
        while True:
            time.sleep(0.05)  # Poll every 50ms
            status = self.check_status()
            if status.get("status") == "complete":
                return status
            elif status.get("status") == "error":
                return status
    
    def check_status(self):
        message = {"command": "status"}
        return self.send_command(message)

def main():
    controller = None
    try:
        controller = ArduinoProfiledServoController()
        print(f"Connected to Arduino on port: {controller.port}")
        print("Commands: ")
        print("  move <id> <pos> [max_vel] [max_accel] - Move servo with profile")
        print("  q - Quit")
        
        while True:
            user_input = input("> ").strip().split()
            
            if not user_input:
                continue
                
            if user_input[0].lower() == 'q':
                break
                
            try:
                if user_input[0] == 'move':
                    if len(user_input) >= 3:
                        servo_id = int(user_input[1])
                        position = int(user_input[2])
                        max_vel = float(user_input[3]) if len(user_input) > 3 else 60
                        max_accel = float(user_input[4]) if len(user_input) > 4 else 30
                        
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
                
    except KeyboardInterrupt:
        print("\nExiting...")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        if controller:
            controller.close()

if __name__ == "__main__":
    main()
