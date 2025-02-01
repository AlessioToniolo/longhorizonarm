# perception.py
import pyrealsense2 as rs
import numpy as np
import cv2
import replicate
import os
from dotenv import load_dotenv
import base64
from io import BytesIO
from PIL import Image

class ScenePerception:
    def __init__(self, enable_visualization=False):
        load_dotenv()
        self._init_realsense()
        self.enable_visualization = enable_visualization

    def _init_realsense(self):
        """Initialize RealSense camera and get intrinsics"""
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        
        # Get camera intrinsics
        color_profile = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        self.fx = self.intrinsics.fx
        self.fy = self.intrinsics.fy
        self.cx = self.intrinsics.ppx
        self.cy = self.intrinsics.ppy

    def _image_to_base64(self, image):
        """Convert OpenCV image to base64 string"""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(image_rgb)
        buffered = BytesIO()
        pil_image.save(buffered, format="JPEG")
        return base64.b64encode(buffered.getvalue()).decode()

    def _get_3d_coords(self, bbox_center, depth_frame):
        """Calculate 3D coordinates from depth data"""
        center_x, center_y = bbox_center
        depth = depth_frame.get_distance(int(center_x), int(center_y))
        X = (center_x - self.cx) * depth / self.fx
        Y = (center_y - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def detect_object(self, query):
        """Detect specific object based on query string"""
        # Capture frames
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        
        # Convert image to base64 for Replicate API
        base64_image = self._image_to_base64(color_image)
        
        # Run Grounding DINO detection
        try:
            output = replicate.run(
                "adirik/grounding-dino:efd10a8ddc57ea28773327e881ce95e20cc1d734c589f7dd01d2036921ed78aa",
                input={
                    "image": f"data:image/jpeg;base64,{base64_image}",
                    "query": query,
                    "box_threshold": 0.2,
                    "text_threshold": 0.2
                }
            )
            
            if not output or not output.get("detections"):
                return None
                
            # Get the first detection (highest confidence)
            detection = output["detections"][0]
            x1, y1, x2, y2 = map(float, detection["bbox"])
            
            # Calculate center point
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Get 3D coordinates
            X, Y, Z = self._get_3d_coords((center_x, center_y), depth_frame)
            
            detected_object = {
                'label': query,
                'confidence': 1.0,  # Grounding DINO doesn't return confidence scores in this format
                'position': (X, Y, Z),
                'box': [int(x1), int(y1), int(x2), int(y2)]
            }
            
            if self.enable_visualization:
                self._visualize(color_image, detected_object)
                
            return detected_object
            
        except Exception as e:
            print(f"Detection error: {e}")
            return None

    def _visualize(self, color_image, detected_object):
        """Visualize detection results"""
        vis = color_image.copy()
        if detected_object:
            x1, y1, x2, y2 = detected_object['box']
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{detected_object['label']} ({detected_object['confidence']:.2f})"
            pos = detected_object['position']
            coords = f"X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}"
            cv2.putText(vis, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(vis, coords, (x1, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow("Detection", vis)
        cv2.waitKey(1)

    def stop(self):
        """Clean up resources"""
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main():
    """
    Test the ScenePerception class with basic object detection.
    Provides a simple command-line interface for testing different queries.
    """
    # Initialize perception system with visualization enabled
    perception = ScenePerception(enable_visualization=True)
    
    try:
        while True:
            # Get user input for object detection
            query = input("\nEnter object to detect (or 'quit' to exit): ").strip()
            
            if query.lower() == 'quit':
                break
                
            # Attempt detection
            print(f"\nDetecting: {query}")
            result = perception.detect_object(query)
            
            # Display results
            if result:
                print("\nDetection Results:")
                print(f"Label: {result['label']}")
                print(f"Position (X,Y,Z): {result['position']}")
                print(f"Bounding Box: {result['box']}")
            else:
                print(f"No {query} detected in scene.")
                
    except KeyboardInterrupt:
        print("\nTest terminated by user")
    finally:
        # Clean up resources
        perception.stop()
        print("\nResources cleaned up")

if __name__ == "__main__":
    main()