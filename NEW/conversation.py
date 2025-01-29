import os
import json
from anthropic import Anthropic
from dotenv import load_dotenv
import serial
import time

load_dotenv()

class ClaudeChat:
    def __init__(self):
        api_key = os.getenv('ANTHROPIC_API_KEY')
        if not api_key:
            raise ValueError("No API key found. Please check your .env file.")
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port as needed
            time.sleep(2)  # Give Arduino time to reset
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise

        self.client = Anthropic(api_key=api_key)
        self.messages = []
        self.system_prompt = """
           You are a helpful AI assistant that will give my 5° of freedom arm and XYZ coordinate to go to based on what I tell it. 
           The coordinate plane is common sense. Say if I told her to go right it would increase the X and so forth what you need 
           to do is take my complicated commands and turn it into Jason commands so you are not going to output any other text 
           whatsoever other than a Jason object in an array there are two types of Jason on object objects you can send one of 
           them will have a field of pos and a value of a string with the coordinates you give it separated by commas. 
           Another Jason object you can send is a delay where the field is delay and then the value is a number in milliseconds 
           for how long the delay should take place please output however many Jason objects as you want and what and then 
           whatever order you determine all in in a array. Here is an example:
            Query: Move the arm to the right and up
            Response: [{"pos": "200, 200, 300"}]

            Query: Go to the left pause a little and then turn 180 and go up
            Response: [{"pos": "100, 200, 300"}, {"delay": 1000}, {"pos": "100, 200, 300"}]
        """

    def send_command(self, command):
        """Send a single command over serial and wait for completion."""
        try:
            self.serial.write(json.dumps(command).encode() + b'\n')
            time.sleep(0.1)  # Small delay between commands
            
            # If it's a delay command, wait for the specified time
            if 'delay' in command:
                time.sleep(command['delay'] / 1000)  # Convert milliseconds to seconds
                
        except Exception as e:
            print(f"Error sending command: {e}")

    def process_claude_response(self, response_text):
        """Process Claude's response and send commands to Arduino."""
        try:
            # Parse the JSON array from Claude's response
            commands = json.loads(response_text)
            
            # Verify it's a list/array
            if not isinstance(commands, list):
                raise ValueError("Expected a JSON array in response")
            
            # Process each command in the array
            for command in commands:
                print(f"Sending command: {command}")
                self.send_command(command)
                
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON response: {e}")
        except Exception as e:
            print(f"Error processing response: {e}")

    def start_chat(self):
        print("Chat started with our robot assistant. Type 'q' to end the conversation.")
        print("---------------------------------------------")

        while True:
            # Get user input
            user_input = input("\nYou: ").strip()
            
            # Check for exit command
            if user_input.lower() == 'q':
                print("\nEnding chat session...")
                self.serial.close()
                break

            # Add user message to conversation history
            self.messages.append({"role": "user", "content": user_input})

            try:
                # Get Claude's response
                response = self.client.messages.create(
                    model="claude-3-sonnet-20240229",
                    max_tokens=1024,
                    system=self.system_prompt,
                    messages=self.messages
                )

                # Extract Claude's response
                claude_response = response.content[0].text
                print("\nProcessing commands:", claude_response)

                # Process and send commands
                #self.process_claude_response(claude_response)

                # Add Claude's response to conversation history
                self.messages.append({"role": "assistant", "content": claude_response})

            except Exception as e:
                print(f"\nError: {str(e)}")
                print("Please try again.")

def main():
    # Start chat session
    chat = ClaudeChat()
    chat.start_chat()

if __name__ == "__main__":
    main()