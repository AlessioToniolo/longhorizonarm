import serial
import serial.tools.list_ports
import json
import time

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

    def move_servo(self, servo_id, position):
        # Clear buffers
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        
        # Send command
        message = {
            "command": "move",
            "servo_id": servo_id,
            "position": position
        }
        cmd = f"{json.dumps(message)}\n"
        self.serial.write(cmd.encode('ascii'))
        
        # Read response with error handling
        try:
            self.serial.flush()
            response = self.serial.readline()
            if response:
                return json.loads(response.decode('ascii'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.serial.reset_input_buffer()
            return {"status": "error", "message": "Invalid response"}
        
        return {"status": "error", "message": "No response"}

    def send_command(self, message):
        """Send a command to the Arduino and get response"""
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        
        cmd = f"{json.dumps(message)}\n"
        self.serial.write(cmd.encode('ascii'))
        
        try:
            self.serial.flush()
            response = self.serial.readline()
            if response:
                return json.loads(response.decode('ascii'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.serial.reset_input_buffer()
            return {"status": "error", "message": "Invalid response"}
        
        return {"status": "error", "message": "No response"}

    def close(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

def main():
    controller = None
    try:
        controller = ArduinoServoController()
        print(f"Connected to Arduino on port: {controller.port}")
        print("Enter servo ID and position (0-270), or 'q' to quit")
        
        while True:
            user_input = input("Servo ID and Position> ").strip()
            
            if user_input.lower() == 'q':
                break
            
            try:
                servo_id, position = map(int, user_input.split())
                if 0 <= position <= 270:
                    print(f"Moving servo {servo_id} to position: {position}")
                    response = controller.move_servo(servo_id, position)
                    print(f"Response: {response}")
                else:
                    print("Position must be between 0 and 270 degrees")
            except ValueError:
                print("Please enter valid servo ID and position, or 'q' to quit")
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if controller:
            controller.close()

if __name__ == "__main__":
    main()