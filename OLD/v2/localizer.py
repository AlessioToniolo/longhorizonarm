import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import threading
import cv2
import numpy as np
from perception.camera.camera_labeler import draw_detection
from perception.camera.depth_map import DepthVisualizer
from perception.ml.vlm_handler import VLMHandler
from perception.camera.realsense_interface import RealSenseInterface

class ObjectLocalizer:
    def __init__(self, min_depth=0.1, max_depth=1.0):
        self.realsense = RealSenseInterface()
        
        self.vlm_handler = VLMHandler(stream=self.realsense)
        
        self.depth_visualizer = DepthVisualizer(min_depth=min_depth, max_depth=max_depth, stream=self.realsense)
        
        self.running = False
        self.target_object = None
        self.object_location = None
    
    def _calculate_object_position(self, bbox, depth_image):
        x1, y1, x2, y2 = map(int, bbox)
        
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        depth = depth_image[center_y, center_x] * self.depth_visualizer.depth_scale
        
        fx = 640 / (2 * np.tan(np.pi/6))  # Approximate focal length
        fy = fx
        
        X = (center_x - 320) * depth / fx
        Y = (center_y - 240) * depth / fy
        Z = depth
        
        self.object_location = (X, Y, Z)
    
    def locate_object(self, object_name):
        self.target_object = object_name
        self.object_location = None
        
        max_attempts = 30  # Wait for max 3 seconds (at 10 fps)
        attempts = 0
        
        while attempts < max_attempts and self.object_location is None:
            attempts += 1
            cv2.waitKey(100)
        
        return self.object_location
    
    def stop(self):
        self.running = False
        self.realsense.stop()
        cv2.destroyAllWindows()

def main():
    # Initialize object localizer
    localizer = ObjectLocalizer(min_depth=0.1, max_depth=2.0)
    
    try:
        # Start visualization
        localizer.start_visualization_threads()
        
        while True:
            # Get object to locate from user
            object_name = input("\nEnter object to locate (or 'q' to quit): ")
            if object_name.lower() == 'q':
                break
            
            # Try to locate the object
            location = localizer.locate_object(object_name)
            
            if location:
                X, Y, Z = location
                print(f"\nFound {object_name} at:")
                print(f"X: {X:.3f} meters (left/right)")
                print(f"Y: {Y:.3f} meters (up/down)")
                print(f"Z: {Z:.3f} meters (forward/backward)")
            else:
                print(f"\nCouldn't find {object_name} in the scene")
    
    finally:
        localizer.stop()

if __name__ == "__main__":
    main()