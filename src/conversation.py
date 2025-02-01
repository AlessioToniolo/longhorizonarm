import os
import json
import time
import threading
import serial
import sys
import speech_recognition as sr
from dotenv import load_dotenv
from openai import OpenAI
from perception import ScenePerception

load_dotenv()

# VoiceRecognizer class remains unchanged
class VoiceRecognizer:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        self.audio_data = None

    def capture_audio(self):
        with self.microphone as source:
            print("\nListening... (Press Enter to stop)")
            self.audio_data = self.recognizer.listen(source)

    def listen(self):
        try:
            thread = threading.Thread(target=self.capture_audio)
            thread.start()
            input()  # Stop listening when user presses Enter
            thread.join()
            if self.audio_data:
                print("Processing speech...")
                text = self.recognizer.recognize_google(self.audio_data)
                print(f"Recognized: {text}")
                return text
        except sr.UnknownValueError:
            print("Could not understand audio")
        except sr.RequestError as e:
            print(f"Speech service error: {e}")
        return None

class OpenAIChat:
    def __init__(self, model_name):
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            raise ValueError("Missing OPENAI_API_KEY in .env file.")
            
        self.model_name = model_name
        self.voice_recognizer = VoiceRecognizer()
        self.scene_perception = ScenePerception(enable_visualization=True)
        self.last_scene_update = 0
        self.last_scene_info = "No objects detected."
        self.scene_cache_duration = 0.3  # seconds

        try:
            self.serial = serial.Serial('COM5', 115200, timeout=1)
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            raise

        self.client = OpenAI(api_key=api_key)
        self.messages = []
        self.system_prompt = """
<<<<<<< HEAD
            Your name is Adap, a robotic assistant controlling a 6-servo robotic arm. The arm consists of the following servos in sequence:
1. Base rotation servo – rotates the entire arm.
2. Shoulder1 servo.
3. Shoulder2 servo.
4. Wrist servo.
5. Twist servo – mounted after the wrist and rotates in the opposite direction.
6. Gripper servo – controls the opening and closing of the gripper.

Each servo accepts angles from 0° to 180°. The gripper servo functions as follows:
  - Fully open: 170° (corresponds to a 5-inch gap between the gripper fingers).
  - Fully closed: 110° (corresponds to a 0-inch gap).
For any desired gap between 0 and 5 inches, compute the gripper angle using linear interpolation between 110° (closed) and 170° (open).
=======
        Your name is Adap, a robot with 6 degrees of freedom. You will respond to user queries with a sequence of servo movements.
        Each movement consists of servo angles (0 to 180) and a delay after the movement.
         
        You have an Intel RealSense camera mounted to the side of your workspace, looking down at about a 45-degree angle.
        The camera detects objects and provides their 3D coordinates in your base frame, where:
        - X: Forward/backward from your base (positive is forward)
        - Y: Left/right from your base (positive is right)
        - Z: Up/down from your base (positive is up)
        
        We have a base rotation servo, a shoulder1 servo, a shoulder2 servo, a wrist servo, a twist servo, and a gripper servo.
        Shoulder1, shoulder2, and wrist are on the same plane.
        
        The first joint is off of the ground by 145.1mm. The first member length is 187mm and the second member length is 162mm 
        and the last member length (from the joint to the end of the gripper) is 86mm.

        Your response should include an array of movement commands enclosed in brackets. Each command should be a JSON object with 
        seven fields: base, shoulder1, shoulder2, wrist, twist, gripper, and delayAfter (in milliseconds). Here's an example of a sequence:
        [
            {"base": 90, "shoulder1": 90, "shoulder2": 90, "wrist": 90, "twist": 90, "gripper": 90, "delayAfter": 1000},
            {"base": 120, "shoulder1": 100, "shoulder2": 80, "wrist": 90, "twist": 90, "gripper": 45, "delayAfter": 500}
        ]
>>>>>>> 17859e4bb6a901f80b66261a9f7d581f8f6acc2b

Your task is to generate a sequence of movement commands based on user queries and scene information. Each command must be a JSON object with the following keys:
  - "base": angle for the base servo.
  - "shoulder1": angle for the first shoulder.
  - "shoulder2": angle for the second shoulder.
  - "wrist": angle for the wrist.
  - "twist": angle for the twist servo.
  - "gripper": angle for the gripper servo (calculated based on the desired gap).
  - "delayAfter": delay (in milliseconds) after executing that command.

For example:
[
  {"base": 90, "shoulder1": 90, "shoulder2": 90, "wrist": 90, "twist": 90, "gripper": 170, "delayAfter": 1000},
  {"base": 120, "shoulder1": 100, "shoulder2": 80, "wrist": 90, "twist": 60, "gripper": 140, "delayAfter": 500}
]

You have an Intel RealSense camera mounted on your end effector (the claw). This camera now provides a robot-centric view with the following coordinate system:
  - X-axis: Points to the right (positive is right).
  - Y-axis: Points downward (positive is down).
  - Z-axis: Points forward (positive is forward).

The camera returns object positions as 3D coordinates (X, Y, Z) in meters relative to its own frame—use these directly when planning your movements.

Additionally, if you need to perform an on-demand vision scan without sending a command to the Arduino, output a special command:
    {"command": "vision_scan"}
This command will trigger the vision system to update object positions and detection scores.

Your responses must consist solely of a JSON array of commands (or a single command for a vision scan) without any extra commentary. Follow the coordinate conventions and command format exactly, and include only the features currently needed.
"""
        # Initialize messages with system prompt
        self.messages = [{"role": "system", "content": self.system_prompt}]

    def get_scene_info(self):
        objects = self.scene_perception.get_scene_objects()
        if not objects:
            return "No objects detected."
        desc = "Detected objects:\n"
        for obj in objects:
            x, y, z = obj['position']
            desc += f"- {obj['label']} (score: {obj['score']:.2f}) at X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}\n"
        return desc

    def get_current_scene(self):
        now = time.time()
        if now - self.last_scene_update > self.scene_cache_duration:
            self.last_scene_info = self.get_scene_info()
            self.last_scene_update = now
        return self.last_scene_info

    def send_command(self, command):
        if command.get("command") == "vision_scan":
            print("Performing vision scan...")
            objects = self.scene_perception.get_scene_objects()
            if not objects:
                print("No objects detected.")
            else:
                for obj in objects:
                    print(f"{obj['label']} at {obj['position']}")
            return

        try:
            self.serial.write(json.dumps(command).encode() + b'\n')
            time.sleep(command.get('delayAfter', 0) / 1000)
        except Exception as e:
            print(f"Error sending command: {e}")

    def process_openai_response(self, response_text):
        try:
            start = response_text.find('[')
            end = response_text.rfind(']')
            if start != -1 and end != -1:
                command_list = json.loads(response_text[start:end+1])
                if isinstance(command_list, list):
                    print(f"Executing {len(command_list)} commands...")
                    for cmd in command_list:
                        print(f"Command: {cmd}")
                        self.send_command(cmd)
                else:
                    print("Response is not a list.")
            else:
                print("No valid commands found in response.")
        except Exception as e:
            print(f"Error processing response: {e}")

    def start_chat(self):
        print("Chat started. Say 'quit' or 'q' to exit.")
        print("Press Enter for keyboard input.")
        while True:
            print("\nPress Enter to type or speak:")
            choice = input().strip().lower()
            if choice == "q":
                break
            user_input = input("Type your message: ").strip() if choice else self.voice_recognizer.listen()
            if not user_input or user_input.lower() in ["quit", "q"]:
                break

            scene_info = self.get_current_scene()
            full_input = f"{user_input}\n\n{scene_info}"
            self.messages.append({"role": "user", "content": full_input})
            
            try:
                response = self.client.chat.completions.create(
                    model=self.model_name,
                    messages=self.messages,
                    max_tokens=1024,
                    temperature=0.7
                )
                openai_response = response.choices[0].message.content
                print("\nResponse:", openai_response)
                self.process_openai_response(openai_response)
                self.messages.append({"role": "assistant", "content": openai_response})
            except Exception as e:
                print("Error:", e)
                print("Try again.")
                
        print("Ending chat...")
        self.scene_perception.stop()
        self.serial.close()

def main():
    # Example model name - replace with your preferred model
    model_name = "o3-mini-2025-01-31"  # or "gpt-3.5-turbo" etc.
    chat = OpenAIChat(model_name)
    chat.start_chat()

if __name__ == "__main__":
    main()