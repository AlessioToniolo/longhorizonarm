import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from perception.camera.depth_map import DepthVisualizer
from perception.ml.vlm_handler import VLMHandler
import cv2
import numpy as np
import time

class CommandGenerator:
    def __init__(self):
        self.vlm_handler = VLMHandler()
        self.depth_vis = DepthVisualizer(min_depth=0.1, max_depth=1.0)
        
        # Stop any existing pipelines
        try:
            self.depth_vis.pipeline.stop()
        except:
            pass
            
        # Start fresh pipeline
        self.depth_vis.pipeline.start(self.depth_vis.config)

    def __del__(self):
        try:
            self.depth_vis.pipeline.stop()
            self.vlm_handler.stop()
        except:
            pass

    def draw_detection(self, frame, result):
        x1, y1, x2, y2 = map(int, result['box'])
        label = result['label']
        score = result['score']
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text = f"{label}: {score:.2f}"
        cv2.putText(frame, text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def get_object_location(self, target_object):
        try:
            frame, results = self.vlm_handler.process_frame()
            frames = self.depth_vis.pipeline.wait_for_frames(timeout_ms=1000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Create depth colormap
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )

            if results and frame is not None:
                # Draw all detections
                for result in results:
                    self.draw_detection(frame, result)
                    if target_object.lower() in result['label'].lower():
                        x1, y1, x2, y2 = map(int, result['box'])
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        depth = depth_image[center_y, center_x] * self.depth_vis.depth_scale
                        x = (center_x - 320) * depth / 640
                        y = (center_y - 240) * depth / 480
                        z = depth
                        
                        # Draw target crosshair
                        cv2.drawMarker(frame, (center_x, center_y), 
                                     (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

                # Show streams
                cv2.imshow('Object Detection', frame)
                cv2.imshow('Depth Map', depth_colormap)
                
                return x, y, z
            return None

        except Exception as e:
            print(f"Error capturing frames: {str(e)}")
            return None

    def run(self):
        try:
            while True:
                target_object = input("\nEnter object to locate (or 'q' to quit): ")
                
                if target_object.lower() == 'q':
                    break
                
                location = self.get_object_location(target_object)
                
                if location:
                    x, y, z = location
                    print(f"Found {target_object} at:")
                    print(f"X: {x:.3f}m (right/left)")
                    print(f"Y: {y:.3f}m (up/down)")
                    print(f"Z: {z:.3f}m (distance from camera)")
                else:
                    print(f"Object '{target_object}' not found")
                
                cv2.waitKey(1)
                time.sleep(0.03)  # Control frame rate
                
        finally:
            self.depth_vis.pipeline.stop()
            self.vlm_handler.stop()
            cv2.destroyAllWindows()

def main():
    command_generator = CommandGenerator()
    command_generator.run()

if __name__ == "__main__":
    main()