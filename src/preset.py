import serial
import json
import time

class RobotPreset:
    def __init__(self, port='COM5', baud_rate=115200):
        """Initialize serial connection for robot presets"""
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None

    def connect(self):
        """Establish serial connection"""
        self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
        time.sleep(2)  # Wait for serial connection to stabilize

    def send_command(self, command):
        """Send a single movement command to robot"""
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial connection not established")
        try:
            self.serial.write(json.dumps(command).encode() + b'\n')
            time.sleep(command.get('delayAfter', 0) / 1000)
        except Exception as e:
            print(f"Error sending preset command: {e}")
            self.close()
            raise

    def run_preset_sequence(self):
        """Run a sequence of preset positions before starting conversation"""
        try:
            self.connect()
            preset_positions = [
                # Home position
                {
                    "base": 90,
                    "shoulder1": 90,
                    "shoulder2": 90,
                    "wrist": 90,
                    "twist": 90,
                    "gripper": 170,
                    "delayAfter": 1000
                },
                # Ready position
                {
                    "base": 90,
                    "shoulder1": 110,
                    "shoulder2": 70,
                    "wrist": 90,
                    "twist": 90,
                    "gripper": 140,
                    "delayAfter": 1000
                }
            ]

            print("Running preset sequence...")
            for position in preset_positions:
                print(f"Moving to position: {position}")
                self.send_command(position)
            print("Preset sequence completed")
        finally:
            self.close()

    def close(self):
        """Close the serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Preset serial connection closed")