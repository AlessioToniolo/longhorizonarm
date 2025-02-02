import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from transformers import pipeline
from typing import List, Dict, Tuple, Optional

class CameraInterface:
    """Handles RealSense camera operations following Single Responsibility Principle"""
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(self.config)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        
        # Get camera intrinsics
        color_stream = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        self.fx = self.intrinsics.fx
        self.fy = self.intrinsics.fy
        self.cx = self.intrinsics.ppx
        self.cy = self.intrinsics.ppy

    def get_frames(self) -> Tuple[np.ndarray, np.ndarray]:
        """Captures and returns the current color and depth frames"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            raise RuntimeError("Failed to capture frames from RealSense camera")
            
        return (
            np.asanyarray(color_frame.get_data()),
            np.asanyarray(depth_frame.get_data())
        )

    def calculate_3d_position(self, x: int, y: int, depth: float) -> Tuple[float, float, float]:
        """Converts 2D pixel coordinates and depth to 3D world coordinates"""
        Z = depth * self.depth_scale
        X = (x - self.cx) * Z / self.fx
        Y = (y - self.cy) * Z / self.fy
        return (X, Y, Z)

    def stop(self):
        """Stops the camera pipeline"""
        self.pipeline.stop()


class VLModel:
    """Handles visual language model operations"""
    def __init__(self, device: str = None):
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            
        # Initialize GroundingDINO using Hugging Face pipeline
        self.detector = pipeline("object-detection", 
                               model="IDEA-Research/GroundingDINO-T",
                               device=self.device)

    def detect_objects(self, image: np.ndarray, text_prompt: str = None) -> List[Dict]:
        """Detects objects in the image using GroundingDINO"""
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Detect objects
        if text_prompt:
            detections = self.detector(image_rgb, text=text_prompt)
        else:
            detections = self.detector(image_rgb)
            
        # Convert detections to our format
        results = []
        for det in detections:
            box = [
                det['box']['xmin'],
                det['box']['ymin'],
                det['box']['xmax'],
                det['box']['ymax']
            ]
            results.append({
                'label': det['label'],
                'score': det['score'],
                'box': box
            })
            
        return results


class ScenePerception:
    """Main class that combines camera and VLM functionality"""
    def __init__(self, enable_visualization: bool = False):
        self.camera = CameraInterface()
        self.vlm = VLModel()
        self.enable_visualization = enable_visualization
        self.current_objects = []

    def get_scene_objects(self, text_prompt: str = None) -> List[Dict]:
        """Returns all detected objects in the current scene"""
        color_image, depth_image = self.camera.get_frames()
        
        # Get detections
        detections = self.vlm.detect_objects(color_image, text_prompt)
        
        # Process detections
        objects = []
        for det in detections:
            x1, y1, x2, y2 = map(int, det['box'])
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get 3D position
            position = self.camera.calculate_3d_position(
                center_x, center_y, depth_image[center_y, center_x]
            )
            
            objects.append({
                'label': det['label'],
                'score': det['score'],
                'position': position,
                'box': [x1, y1, x2, y2]
            })
        
        self.current_objects = objects
        
        if self.enable_visualization:
            self._visualize(color_image, depth_image, objects)
            
        return objects

    def find_object(self, target_label: str) -> Optional[Dict]:
        """Finds a specific object in the scene"""
        objects = self.get_scene_objects(text_prompt=target_label)
        if objects:
            # Return the highest confidence detection
            return max(objects, key=lambda x: x['score'])
        return None

    def _visualize(self, color_image: np.ndarray, depth_image: np.ndarray, 
                  objects: List[Dict]):
        """Visualizes detections and depth map"""
        vis = color_image.copy()
        
        # Draw detections
        for obj in objects:
            x1, y1, x2, y2 = obj['box']
            # Draw bounding box
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Add labels
            label = f"{obj['label']} ({obj['score']:.2f})"
            pos = obj['position']
            coords = f"X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}"
            cv2.putText(vis, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(vis, coords, (x1, y1 + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Show depth map
        depth_vis = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        
        cv2.imshow("Detections", vis)
        cv2.imshow("Depth Map", depth_vis)
        cv2.waitKey(1)

    def stop(self):
        """Cleanup resources"""
        self.camera.stop()
        cv2.destroyAllWindows()


# Example usage
if __name__ == "__main__":
    perception = ScenePerception(enable_visualization=True)
    try:
        while True:
            objects = perception.get_scene_objects()
            # Process objects as needed
            
    except KeyboardInterrupt:
        perception.stop()