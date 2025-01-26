import serial
import serial.tools.list_ports
import json
import time

MAX_VEL = 60  # Maximum velocity in deg/sec
MAX_ACCEL = 45  # Maximum acceleration in deg/sec²

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
        self.servo_positions = {}  # Track positions

    def _find_arduino_port(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if "CH340" in port.description or "Arduino" in port.description:
                return port.device
        return ports[0].device if ports else None

    def get_current_position(self, servo_id):
        return self.servo_positions.get(servo_id, 90)  # Default to center position

    def move_servo(self, servo_id, position):
        # Update position tracking
        self.servo_positions[servo_id] = position
        
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

    def move_multiple_servos(self, movements):
        """
        Move multiple servos simultaneously
        movements: list of dicts with servo_id and position
        """
        message = {
            "command": "move_multiple",
            "movements": movements
        }
        return self.send_command(message)

    def send_command(self, message):
        """Send a command to the Arduino and get response"""
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        
        cmd = f"{json.dumps(message)}\n"
        self.serial.write(cmd.encode('ascii'))
        time.sleep(0.1)  # Add small delay for Arduino processing
        
        try:
            self.serial.flush()
            response = self.serial.readline()
            if response:
                return json.loads(response.decode('ascii'))
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            self.serial.reset_input_buffer()
            return {"status": "error", "message": f"Invalid response: {str(e)}"}
        
        return {"status": "error", "message": "No response"}

    def move_with_trapezoidal_profile(self, servo_id, target_position):
        current_position = self.get_current_position(servo_id)
        distance = target_position - current_position
        direction = 1 if distance > 0 else -1
        
        # Time calculations
        accel_time = MAX_VEL / MAX_ACCEL  # 2 seconds with our values
        accel_distance = 0.5 * MAX_ACCEL * accel_time ** 2  # 60 degrees with our values

        # Determine if we need constant velocity phase
        if abs(distance) < 2 * accel_distance:
            # Short move - triangular profile
            accel_time = (abs(distance) / MAX_ACCEL) ** 0.5
            accel_distance = 0.5 * MAX_ACCEL * accel_time ** 2
            constant_time = 0
        else:
            # Long move - trapezoidal profile
            constant_distance = abs(distance) - 2 * accel_distance
            constant_time = constant_distance / MAX_VEL

        # Execute movement
        start_time = time.time()
        current_time = 0

        # Acceleration phase
        while current_time < accel_time:
            current_time = time.time() - start_time
            distance_covered = 0.5 * MAX_ACCEL * current_time ** 2
            position = current_position + (direction * distance_covered)
            self.move_servo(servo_id, position)
            time.sleep(0.01)  # 10ms control loop

        # Constant velocity phase
        if constant_time > 0:
            start_constant = time.time()
            while (time.time() - start_constant) < constant_time:
                current_time = time.time() - start_time
                position = current_position + (direction * (accel_distance + 
                          MAX_VEL * (current_time - accel_time)))
                self.move_servo(servo_id, position)
                time.sleep(0.01)

        # Deceleration phase
        start_decel = time.time()
        while (time.time() - start_decel) < accel_time:
            current_time = time.time() - start_time
            total_time = 2 * accel_time + constant_time
            remaining_time = total_time - current_time
            distance_covered = abs(distance) - (0.5 * MAX_ACCEL * remaining_time ** 2)
            position = current_position + (direction * distance_covered)
            self.move_servo(servo_id, position)
            time.sleep(0.01)

        # Final position
        self.move_servo(servo_id, target_position)

    def close(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

def main():
    controller = None
    try:
        controller = ArduinoServoController()
        print(f"Connected to Arduino on port: {controller.port}")
        print("Commands: ")
        print("  single <id> <pos> - Move single servo")
        print("  multi <id1> <pos1> <id2> <pos2> ... - Move multiple servos")
        print("  q - Quit")
        
        while True:
            user_input = input("> ").strip().split()
            
            if not user_input:
                continue
                
            if user_input[0].lower() == 'q':
                break
                
            try:
                if user_input[0] == 'single':
                    servo_id, position = map(int, user_input[1:3])
                    if 0 <= position <= 270:
                        print(f"Moving servo {servo_id} to position: {position}")
                        #response = controller.move_servo(servo_id, position)
                        response = controller.move_with_trapezoidal_profile(servo_id, position)
                        print(f"Response: {response}")
                    else:
                        print("Position must be between 0 and 270 degrees")
                
                elif user_input[0] == 'multi':
                    movements = []
                    for i in range(1, len(user_input), 2):
                        servo_id = int(user_input[i])
                        position = int(user_input[i+1])
                        if not (0 <= position <= 270):
                            print(f"Position {position} out of range")
                            break
                        movements.append({"servo_id": servo_id, "position": position})
                    if movements:
                        response = controller.move_multiple_servos(movements)
                        print(f"Response: {response}")
                
            except (ValueError, IndexError):
                print("Invalid input format")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        if controller:
            controller.close()

if __name__ == "__main__":
    main()