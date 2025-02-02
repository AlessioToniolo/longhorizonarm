import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from pathlib import Path
from groundingdino.util.inference import load_model, load_image, predict
from segment_anything import sam_model_registry, SamPredictor
from typing import List, Dict, Tuple, Optional

DEFAULT_WEIGHTS_DIR = Path("weights")
GROUNDINGDINO_CONFIG_PATH = DEFAULT_WEIGHTS_DIR / "groundingdino" / "config" / "GroundingDINO_SwinT_OGC.py"
GROUNDINGDINO_WEIGHTS_PATH = DEFAULT_WEIGHTS_DIR / "groundingdino" / "weights" / "groundingdino_swint_ogc.pth"
SAM_WEIGHTS_PATH = DEFAULT_WEIGHTS_DIR / "sam" / "sam_vit_h_4b8939.pth"

def setup_weights_directory():
    """Creates the directory structure for model weights if it doesn't exist"""
    # Create main weights directory
    DEFAULT_WEIGHTS_DIR.mkdir(exist_ok=True)
    
    # Create subdirectories for each model
    (DEFAULT_WEIGHTS_DIR / "groundingdino" / "config").mkdir(parents=True, exist_ok=True)
    (DEFAULT_WEIGHTS_DIR / "groundingdino" / "weights").mkdir(parents=True, exist_ok=True)
    (DEFAULT_WEIGHTS_DIR / "sam").mkdir(parents=True, exist_ok=True)
    
    # Check if weights exist
    missing_files = []
    if not GROUNDINGDINO_CONFIG_PATH.exists():
        missing_files.append("GroundingDINO config")
    if not GROUNDINGDINO_WEIGHTS_PATH.exists():
        missing_files.append("GroundingDINO weights")
    if not SAM_WEIGHTS_PATH.exists():
        missing_files.append("SAM weights")
    
    if missing_files:
        raise FileNotFoundError(
            f"Missing required files: {', '.join(missing_files)}. "
            f"Please download the required files and place them in the following locations:\n"
            f"1. GroundingDINO config: {GROUNDINGDINO_CONFIG_PATH}\n"
            f"2. GroundingDINO weights: {GROUNDINGDINO_WEIGHTS_PATH}\n"
            f"3. SAM weights: {SAM_WEIGHTS_PATH}\n"
            f"Download links:\n"
            f"- GroundingDINO: https://github.com/IDEA-Research/GroundingDINO\n"
            f"- SAM: https://github.com/facebookresearch/segment-anything"
)

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
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        # Ensure weights directory is set up
        setup_weights_directory()
            
        # Initialize Grounding-SAM-2
        self.model = load_model(
            str(GROUNDINGDINO_CONFIG_PATH),
            str(GROUNDINGDINO_WEIGHTS_PATH)
        )
        self.sam = sam_model_registry["vit_h"](checkpoint=str(SAM_WEIGHTS_PATH))
        self.sam.to(device=self.device)
        self.predictor = SamPredictor(self.sam)

    def detect_objects(self, image: np.ndarray, text_prompt: str = "all objects", 
                      box_threshold: float = 0.35, text_threshold: float = 0.25) -> Tuple[List[Dict], np.ndarray]:
        """Detects objects in the image using Grounding-SAM-2"""
        # Prepare image for model
        image_source = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.predictor.set_image(image_source)
        
        # Get detections
        boxes, logits, phrases = predict(
            model=self.model,
            image=image_source,
            caption=text_prompt,
            box_threshold=box_threshold,
            text_threshold=text_threshold
        )
        
        # Get segmentation masks for each detection
        masks = []
        for box in boxes:
            masks.append(self.predictor.predict(box=box))
        
        return boxes, logits, phrases, masks


class ScenePerception:
    """Main class that combines camera and VLM functionality"""
    def __init__(self, enable_visualization: bool = False):
        self.camera = CameraInterface()
        self.vlm = VLModel()
        self.enable_visualization = enable_visualization
        self.current_objects = []

    def get_scene_objects(self, text_prompt: str = "all objects") -> List[Dict]:
        """Returns all detected objects in the current scene"""
        color_image, depth_image = self.camera.get_frames()
        
        # Get detections and masks
        boxes, scores, labels, masks = self.vlm.detect_objects(color_image, text_prompt)
        
        # Process detections
        objects = []
        for box, score, label, mask in zip(boxes, scores, labels, masks):
            x1, y1, x2, y2 = map(int, box)
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get 3D position
            position = self.camera.calculate_3d_position(
                center_x, center_y, depth_image[center_y, center_x]
            )
            
            objects.append({
                'label': label,
                'score': float(score),
                'position': position,
                'box': [x1, y1, x2, y2],
                'mask': mask
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
            
            # Draw mask
            if 'mask' in obj:
                mask = obj['mask'].astype(np.uint8) * 255
                vis = cv2.addWeighted(
                    vis, 1, 
                    cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 0.5,
                    0
                )
            
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