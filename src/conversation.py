import os
import json
import time
import serial
import sys
from dotenv import load_dotenv
from anthropic import Anthropic
from perception import ScenePerception

load_dotenv()

class RobotChat:
    def __init__(self):
        self._init_apis()
        self._init_hardware()
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
Shoulder1, shoulder2, and wrist are on the same plane. Gripper is technically on the same plane as base, and twist is on a plane perpendicular to the shoulder and wrist plane.

The first joint is off of the ground by 145.1mm. The first member length is 187mm and the second member length is 162mm 
and the last member length is 86mm.

You can search for specific objects by describing them in natural language. When you receive a query about an object,
the system will try to locate it and provide its 3D coordinates if found.

Your response should include an array of movement commands enclosed in brackets. Each command should be a JSON object with 
these fields: base, shoulder1, shoulder2, wrist, twist, gripper, and delayAfter (in milliseconds). Example:
[
    {"base": 90, "shoulder1": 90, "shoulder2": 90, "wrist": 90, "twist": 90, "gripper": 170, "delayAfter": 1000},
    {"base": 120, "shoulder1": 100, "shoulder2": 80, "wrist": 90, "twist": 60, "gripper": 110, "delayAfter": 500}
]

(make sure to not add comments in between each command, because it will break my python code to process)

Gripper open is 170, closed is 110, with linear interpolation between.
The twist servo rotates the end effector on a plane perpendicular to other joints.

When asked about objects, you'll receive information about the specific object detected, including its 3D position.
The camera is mounted on the end effector, so coordinates are relative to your end effector position."""

        self.query_extraction_prompt = """You are a natural language processing expert focused on extracting object queries from user input. Your task is to identify if a user is asking about finding, locating, or interacting with an object, and extract the object description.

Rules:
1. If the user is asking about finding or interacting with an object, return a JSON object with:
   - "has_query": true
   - "object_description": detailed description of the object
   - "action": the intended action (find, pick up, move, etc.)
2. If the user is not asking about an object, return:
   - "has_query": false
   - "object_description": null
   - "action": null

Examples:

Input: "Can you find the blue coffee mug?"
Output: {
    "has_query": true,
    "object_description": "blue coffee mug",
    "action": "find"
}

Input: "Pick up the red pen next to the notebook"
Output: {
    "has_query": true,
    "object_description": "red pen near notebook",
    "action": "pick up"
}

Input: "Wave hello"
Output: {
    "has_query": false,
    "object_description": null,
    "action": null
}

Input: "Move your base joint to 90 degrees"
Output: {
    "has_query": false,
    "object_description": null,
    "action": null
}

Respond only with the JSON object, no additional text or explanation."""

    def _init_apis(self):
        """Initialize API clients and perception system"""
        api_key = os.getenv('ANTHROPIC_API_KEY')
        if not api_key:
            raise ValueError("Missing ANTHROPIC_API_KEY in .env file")
        self.client = Anthropic(api_key=api_key)
        self.scene_perception = ScenePerception(enable_visualization=True)

    def _init_hardware(self):
        """Initialize serial connection"""
        try:
            self.serial = serial.Serial('COM5', 115200, timeout=1)
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            raise

    def _extract_object_query(self, user_input):
        """Extract object query from user input using Claude's NLP capabilities"""
        try:
            # Get NLP analysis from Claude
            response = self.client.messages.create(
                model="claude-3-5-sonnet-20241022",
                max_tokens=150,  # Small response size since we only need the JSON
                messages=[{
                    "role": "user",
                    "content": user_input
                }],
                system=self.query_extraction_prompt
            )
            
            # Parse the response
            try:
                result = json.loads(response.content[0].text)
                if result["has_query"]:
                    return result["object_description"]
                return None
            except json.JSONDecodeError:
                print("Error parsing Claude's query extraction response")
                return None
                
        except Exception as e:
            print(f"Error in query extraction: {e}")
            return None

    def get_scene_info(self, query=None):
        """Get scene information for a specific query"""
        if not query:
            return "No specific object queried."
            
        detected_object = self.scene_perception.find_object(query)
        if not detected_object:
            return f"Could not find {query} in the scene."
            
        x, y, z = detected_object['position']
        return f"Found {query} (confidence: {detected_object['score']:.2f}) at X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}"

    def send_command(self, command):
        """Send movement command to robot"""
        try:
            self.serial.write(json.dumps(command).encode() + b'\n')
            time.sleep(command.get('delayAfter', 0) / 1000)
        except Exception as e:
            print(f"Error sending command: {e}")

    def process_claude_response(self, response_text):
        """Process and execute Claude's movement commands"""
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
        """Start the interactive chat session"""
        print("Chat started. Type 'quit' or 'q' to exit.")
        
        while True:
            user_input = input("\nEnter your message: ").strip()
            if user_input.lower() in ["quit", "q"]:
                break

            # Extract object query using NLP
            query = self._extract_object_query(user_input)
            scene_info = self.get_scene_info(query) if query else ""
            
            full_input = f"{user_input}\n\n{scene_info}" if scene_info else user_input
            self.messages.append({"role": "user", "content": full_input})
            
            try:
                response = self.client.messages.create(
                    model="claude-3-5-sonnet-20241022",
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
    chat = RobotChat()
    chat.start_chat()

if __name__ == "__main__":
    main()