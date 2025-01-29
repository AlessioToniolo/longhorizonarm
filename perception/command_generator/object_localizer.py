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
        # Initialize shared RealSense interface
        self.realsense = RealSenseInterface()
        
        # Initialize components with shared interface
        self.vlm_handler = VLMHandler(stream=self.realsense)
        
        self.depth_visualizer = DepthVisualizer(min_depth=min_depth, max_depth=max_depth, stream=self.realsense)
        
        self.running = False
        self.target_object = None
        self.object_location = None
        
    def start_visualization_threads(self):
        """Start visualization thread"""
        self.running = True
        self.vis_thread = threading.Thread(target=self._run_visualization)
        self.vis_thread.daemon = True
        self.vis_thread.start()
    
    def _run_visualization(self):
        """Run both visualizations using shared interface"""
        while self.running:
            # Get frames from shared interface
            depth_image, color_image = self.depth_visualizer.process_frames()
            if depth_image is None or color_image is None:
                continue
            
            # Process frames for object detection
            results = self.vlm_handler.vlm_model.predict(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
            
            # Create detection visualization
            detection_vis = color_image.copy()
            if results:
                for result in results:
                    draw_detection(detection_vis, result)
                    if self.target_object and result['label'].lower() == self.target_object.lower():
                        self._calculate_object_position(result['box'], depth_image)
            
            # Create depth visualization
            points_y, points_x, depth_values = self.depth_visualizer.get_depth_points(depth_image)
            depth_vis = self.depth_visualizer.draw_depth_dots(color_image, points_y, points_x, depth_values)
            
            # Show both visualizations
            cv2.imshow('Object Detection', detection_vis)
            cv2.imshow('Depth Visualization', depth_vis)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
    
    def _calculate_object_position(self, bbox, depth_image):
        """Calculate the 3D position of detected object"""
        x1, y1, x2, y2 = map(int, bbox)
        
        # Get center point of bounding box
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        # Calculate depth at center point (in meters)
        depth = depth_image[center_y, center_x] * self.depth_visualizer.depth_scale
        
        # Convert to camera coordinates (assuming camera intrinsics)
        fx = 640 / (2 * np.tan(np.pi/6))  # Approximate focal length
        fy = fx
        
        # Calculate X and Y coordinates
        X = (center_x - 320) * depth / fx
        Y = (center_y - 240) * depth / fy
        Z = depth
        
        self.object_location = (X, Y, Z)
    
    def locate_object(self, object_name):
        """Main method to locate specific object in the scene"""
        self.target_object = object_name
        self.object_location = None
        
        max_attempts = 30  # Wait for max 3 seconds (at 10 fps)
        attempts = 0
        
        while attempts < max_attempts and self.object_location is None:
            attempts += 1
            cv2.waitKey(100)
        
        return self.object_location
    
    def stop(self):
        """Clean up and stop all processes"""
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