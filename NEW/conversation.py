import speech_recognition as sr
import os
import json
from anthropic import Anthropic
from dotenv import load_dotenv
import serial
import time
import threading
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from perception.ml.vlm_handler import VLMHandler

load_dotenv()

class VoiceRecognizer:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.is_listening = False
        self.audio_data = None
        
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
    
    def capture_audio(self):
        with self.microphone as source:
            print("\nListening... (Press Enter to stop)")
            self.audio_data = self.recognizer.listen(source)
    
    def listen(self):
        try:
            audio_thread = threading.Thread(target=self.capture_audio)
            audio_thread.start()
            input()
            audio_thread.join()
            
            if self.audio_data:
                print("Processing speech...")
                text = self.recognizer.recognize_google(self.audio_data)
                print(f"Recognized: {text}")
                return text
                
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
            return None

class ClaudeChat:
    def __init__(self):
        api_key = os.getenv('ANTHROPIC_API_KEY')
        if not api_key:
            raise ValueError("No API key found. Please check your .env file.")
        
        self.voice_recognizer = VoiceRecognizer()
        
        try:
            self.serial = serial.Serial('COM5', 115200, timeout=1)
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise

        self.client = Anthropic(api_key=api_key)
        self.messages = []
        self.system_prompt = """
        Your name is Adap, a robot with 4 degrees of freedom. You will respond to user queries with a sequence of servo movements.
        Each movement consists of servo angles (0 to 180) and a delay after the movement.
         
        Left is right and up is down. Directions are flipped from what I tell you.

        We have a base rotation servo, a shoulder1 servo, a shoulder2 servo, and a wrist servo.
        Shoulder1, shoulder2, and wrist are on the same plane.
        
        The first joint is off of the ground by 145.1mm. The first member length is 187mm and the second member length is 162mm 
        and the last member length is 86mm.

        Your response should include an array of movement commands enclosed in brackets. Each command should be a JSON object with 
        five fields: base, shoulder1, shoulder2, wrist, and delayAfter (in milliseconds). Here's an example of a sequence:
        [
            {"base": 90, "shoulder1": 90, "shoulder2": 90, "wrist": 90, "delayAfter": 1000},
            {"base": 120, "shoulder1": 100, "shoulder2": 80, "wrist": 90, "delayAfter": 500}
        ]

        Also, you will be given a list of objects coordinates, labels, and prediction scores. If the user asks you to pick up an object,
        you can choose from the list and use their coordinates for the kinematics calculations.
        """

    def send_command(self, command):
        try:
            self.serial.write(json.dumps(command).encode() + b'\n')
            time.sleep(command.get('delayAfter', 0) / 1000)  # Convert ms to seconds
        except Exception as e:
            print(f"Error sending command: {e}")

    def process_claude_response(self, response_text):
        try:
            # Find the array of commands between square brackets
            start = response_text.find('[')
            end = response_text.rfind(']')
            
            if start != -1 and end != -1:
                json_str = response_text[start:end + 1]
                commands = json.loads(json_str)
                
                if isinstance(commands, list):
                    print(f"Executing {len(commands)} commands...")
                    for command in commands:
                        print(f"Command: {command}")
                        self.send_command(command)
                else:
                    print("Invalid command format - expected an array of commands")
            else:
                print("No valid commands found in response")
                
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON commands: {e}")
        except Exception as e:
            print(f"Error processing response: {e}")

    def start_chat(self):
        print("Chat started with our robot assistant.")
        print("Say 'quit' or type 'q' to end the conversation.")
        print("Say 'type' or press Enter for keyboard input.")
        print("---------------------------------------------")

        while True:
            print("\nPress Enter to type, or just start speaking:")
            key_input = input().strip().lower()
            
            if key_input == "q":
                break
                
            if key_input:
                user_input = input("Type your message: ").strip()
            else:
                user_input = self.voice_recognizer.listen()
                
            if not user_input or user_input.lower() in ['quit', 'q']:
                print("\nEnding chat session...")
                self.serial.close()
                break

            self.messages.append({"role": "user", "content": user_input})

            try:
                response = self.client.messages.create(
                    model="claude-3-5-sonnet-latest",
                    max_tokens=1024,
                    system=self.system_prompt,
                    messages=self.messages
                )

                claude_response = response.content[0].text
                print("\nProcessing commands:", claude_response)

                self.process_claude_response(claude_response)
                self.messages.append({"role": "assistant", "content": claude_response})

            except Exception as e:
                print(f"\nError: {str(e)}")
                print("Please try again.")
    
def main():
    chat = ClaudeChat()
    chat.start_chat()

if __name__ == "__main__":
    main()