import threading
import signal
import sys
from perception import ScenePerception
from conversation import RobotChat

def run_perception(perception):
    """Run continuous object detection in a separate thread"""
    try:
        while True:
            perception.get_scene_objects()  # This will update visualization windows
    except KeyboardInterrupt:
        print("\nStopping perception system...")
    finally:
        perception.stop()

def signal_handler(sig, frame):
    """Handle graceful shutdown on Ctrl+C"""
    print("\nShutting down...")
    sys.exit(0)

def main():
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Initialize perception system with visualization enabled
        perception = ScenePerception(enable_visualization=True)
        print("Initialized perception system")
        
        # Start perception in a separate thread
        perception_thread = threading.Thread(
            target=run_perception, 
            args=(perception,),
            daemon=True  # Thread will be terminated when main program exits
        )
        perception_thread.start()
        print("Started perception thread")
        
        # Initialize and start robot chat
        print("Initializing robot chat...")
        chat = RobotChat()
        chat.start_chat()  # This will run until user exits
        
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # Cleanup
        print("\nCleaning up...")
        if 'perception' in locals():
            perception.stop()
        if 'chat' in locals():
            if hasattr(chat, 'serial'):
                chat.serial.close()
        print("Shutdown complete")

if __name__ == "__main__":
    main()