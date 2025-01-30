import sys
import os
import time
import re
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from anthropic import Anthropic
from perception.command_generator.object_localizer import ObjectLocalizer
from dotenv import load_dotenv
load_dotenv()

class LLMHandler:
    def __init__(self):
        """Initialize with ObjectLocalizer and start its streams"""
        self.object_localizer = ObjectLocalizer()
        # Initialize Claude API client
        api_key = os.getenv("ANTHROPIC_API_KEY")
        if not api_key:
            raise ValueError("ANTHROPIC_API_KEY environment variable not set")
        self.client = Anthropic(api_key=api_key)
        
        # Start the visualization threads immediately
        self.object_localizer.start_visualization_threads()
        # Give streams time to initialize
        time.sleep(2)  # Wait for streams to fully start
        
    def extract_objects_from_query(self, query):
        """
        Process natural language query to extract object names using Claude
        
        Args:
            query (str): Natural language query about objects in the scene
            
        Returns:
            list: List of object names to locate
        """
        prompt = f"""
        Extract only the object names that need to be located from this query. Return them as a Python list of strings.
        Only include actual objects mentioned, not pronouns or references.
        Format the response as a Python list like: ["object1", "object2"]
        
        Query: {query}
        
        Examples:
        "Where is the cup and the keyboard?" -> ["cup", "keyboard"]
        "Can you find my phone on the desk?" -> ["phone"]
        "What's the location of the water bottle and mouse pad?" -> ["water bottle", "mouse pad"]
        "I can't see it anywhere" -> []
        "Is there anything on the table?" -> []
        """
        
        response = self.client.messages.create(
            model="claude-3-opus-20240229",
            max_tokens=100,
            temperature=0,
            messages=[{
                "role": "user",
                "content": prompt
            }]
        )
        
        # Extract the Python list from the response
        try:
            # Use regex to find content between square brackets
            match = re.search(r'\[(.*?)\]', response.content)
            if match:
                # Convert string representation of list to actual list
                objects = eval(f"[{match.group(1)}]")
                return objects
            return []
        except:
            return []
            
    def format_location_response(self, object_locations):
        """
        Generate response about object locations
        
        Args:
            object_locations (dict): Dictionary mapping objects to their (x,y,z) coordinates or None
            
        Returns:
            str: Response about object locations
        """
        responses = []
        for obj, location in object_locations.items():
            if location:
                x, y, z = location
                responses.append(f"Found {obj} at coordinates: ({x:.2f}, {y:.2f}, {z:.2f}) meters")
            else:
                responses.append(f"Could not locate {obj} in the scene")
                
        return "\n".join(responses)

    def get_object_locations(self, query):
        """
        Get object locations based on natural language query
        
        Args:
            query (str): Natural language query about objects
            
        Returns:
            dict: Dictionary mapping objects to their (x,y,z) coordinates or None
        """
        objects = self.extract_objects_from_query(query)
        locations = {}
        
        # Get location for each object
        for obj in objects:
            location = self.object_localizer.locate_object(obj)
            locations[obj] = location
            
        return locations

    def process_query(self, query):
        """
        Process natural language query and return formatted response
        
        Args:
            query (str): Natural language query about objects
            
        Returns:
            str: Response about object locations
        """
        locations = self.get_object_locations(query)
        response = self.format_location_response(locations)
        return response

    def cleanup(self):
        """Clean up resources"""
        if hasattr(self, 'object_localizer'):
            self.object_localizer.stop()

def main():
    print("\nInitializing Object Localization System...")
    print("----------------------------------------")
    print("Starting camera streams and detection models...")
    
    try:
        # Initialize handler (this will also start the streams)
        handler = LLMHandler()
        
        print("System ready!")
        print("\nNatural Language Object Localization Demo")
        print("----------------------------------------")
        print("Ask questions like:")
        print("- Where is the cup and keyboard?")
        print("- Can you find my phone?")
        print("- What's the location of the bottle?\n")
        
        while True:
            query = input("Enter your question (or 'q' to quit): ")
            if query.lower() == 'q':
                break
                
            response = handler.process_query(query)
            print(f"\n{response}\n")
            
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    finally:
        handler.cleanup()

if __name__ == "__main__":
    main()