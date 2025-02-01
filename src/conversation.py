import os
import json
import time
import threading
import serial
import sys
import speech_recognition as sr
from dotenv import load_dotenv
from anthropic import Anthropic
from perception import ScenePerception

load_dotenv()

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

class ClaudeChat:
    def __init__(self):
        api_key = os.getenv('ANTHROPIC_API_KEY')
        if not api_key:
            raise ValueError("Missing ANTHROPIC_API_KEY in .env file.")
            
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

        self.client = Anthropic(api_key=api_key)
        self.messages = []
        self.system_prompt = """
 Your name is Adap, a robot with 4 degrees of freedom. You will respond to user queries with a sequence of servo movements.
        Each movement consists of servo angles (0 to 180) and a delay after the movement.
         
        You have an Intel RealSense camera mounted to the side of your workspace, looking down at about a 45-degree angle.
        The camera detects objects and provides their 3D coordinates in your base frame, where:
        - X: Forward/backward from your base (positive is forward)
        - Y: Left/right from your base (positive is right)
        - Z: Up/down from your base (positive is up)
        
        We have a base rotation servo, a shoulder1 servo, a shoulder2 servo, a wrist servo, a twist servo, and a gripper servo.
        Shoulder1, shoulder2, and wrist are on the same plane. Gripper is technically on the same plane as base, and twist is on a plane perp to the shoulder and wrist plane
        
        The first joint is off of the ground by 145.1mm. The first member length is 187mm and the second member length is 162mm 
        and the last member length is 86mm.

        Your response should include an array of movement commands enclosed in brackets. Each command should be a JSON object with 
        five fields: base, shoulder1, shoulder2, wrist, and delayAfter (in milliseconds). Here's an example of a sequence:
        [
            {"base": 90, "shoulder1": 90, "shoulder2": 90, "wrist": 90, "twist": 90, "gripper": 170, "delayAfter": 1000},
            {"base": 120, "shoulder1": 100, "shoulder2": 80, "wrist": 90, "twist": 60, "gripper": 110, "delayAfter": 500}
        ]

        Gripper open is 170 gripper all the way closed is 110 there is about 5 inches in between open and close and it interpolates lienarly between that.

        The twist servo is a rotation servo on a different place perpendicular to the other joints and it rotates the end effector. 

        When I ask about objects, you'll be given a list of detected objects with their positions. The camerta is mounted on the end effector so the reference of the camera is to your forward kinematics ending, all the way at the end effector is the cameras origin. do the calculations as you need for when picking up stuff.
        Use these coordinates to plan your movements when interacting with objects."""
        # Initialize messages with system prompt
        self.messages = []

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

    def process_claude_response(self, response_text):
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
                response = self.client.messages.create(
                    model="claude-3-5-sonnet-latest",
                    max_tokens=1024,
                    messages=self.messages,
                    system=self.system_prompt
                )
                claude_response = response.content[0].text
                print("\nResponse:", claude_response)
                self.process_claude_response(claude_response)
                self.messages.append({"role": "assistant", "content": claude_response})
            except Exception as e:
                print("Error:", e)
                print("Try again.")
                
        print("Ending chat...")
        self.scene_perception.stop()
        self.serial.close()

def main():
    chat = ClaudeChat()
    chat.start_chat()

if __name__ == "__main__":
    main()