import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from transformers import AutoModelForObjectDetection, DetrImageProcessor

class ScenePerception:
    def __init__(self, enable_visualization=False):
        """
        Initialize scene perception with RealSense camera and VLM model
        Args:
            enable_visualization (bool): Whether to show debug visualizations
        """
        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.profile = self.pipeline.start(self.config)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        
        # Initialize VLM
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.model = AutoModelForObjectDetection.from_pretrained("facebook/detr-resnet-50").to(self.device)
        
        # Camera parameters (assuming default RealSense intrinsics)
        self.fx = 615.7725830078125
        self.fy = 615.8447875976562
        self.cx = 326.4713134765625
        self.cy = 240.8659210205078
        
        # Fixed transform from camera to robot base
        # Assuming camera is mounted 0.45m (1.5ft) to the side and looking down
        # You'll need to adjust these values based on your actual setup
        camera_x = 0.45  # 1.5 feet to the side (adjust sign based on which side)
        camera_y = 0.45  # 1.5 feet up
        camera_z = 0.0   # In line with robot base front/back
        camera_tilt = np.radians(45)  # 45 degree downward tilt
        
        # Create transform matrix
        self.T_cam_to_robot = np.array([
            [np.cos(camera_tilt), 0, np.sin(camera_tilt), camera_x],
            [0, 1, 0, camera_y],
            [-np.sin(camera_tilt), 0, np.cos(camera_tilt), camera_z],
            [0, 0, 0, 1]
        ])
        
        self.enable_visualization = enable_visualization

    def transform_point(self, point_camera):
        """Transform point from camera frame to robot frame"""
        # Convert point to homogeneous coordinates
        point_hom = np.array([*point_camera, 1.0])
        
        # Transform point
        point_robot_hom = self.T_cam_to_robot @ point_hom
        
        # Convert back to 3D coordinates
        point_robot = point_robot_hom[:3] / point_robot_hom[3]
        
        return point_robot

    def get_scene_objects(self):
        """
        Get all objects in the current scene with their 3D positions in robot frame
        """
        # Get frames
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return []
            
        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Process image for VLM
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb_image, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        
        # Run inference
        with torch.no_grad():
            outputs = self.model(**inputs)
        
        # Post-process results
        target_sizes = torch.tensor([rgb_image.shape[:2]]).to(self.device)
        results = self.processor.post_process_object_detection(
            outputs,
            target_sizes=target_sizes,
            threshold=0.5
        )[0]
        
        # Process each detection
        scene_objects = []
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box_coords = box.cpu().numpy()
            x1, y1, x2, y2 = map(int, box_coords)
            
            # Get center point
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get depth at center point (in meters)
            depth = depth_image[center_y, center_x] * self.depth_scale
            
            # Calculate camera frame coordinates
            X = (center_x - self.cx) * depth / self.fx
            Y = (center_y - self.cy) * depth / self.fy
            Z = depth
            
            # Transform to robot frame
            robot_pos = self.transform_point((X, Y, Z))
            
            object_info = {
                'label': self.model.config.id2label[label.item()],
                'score': score.cpu().numpy(),
                'position': tuple(robot_pos),
                'box': box_coords
            }
            scene_objects.append(object_info)
            
        # Optional visualization
        if self.enable_visualization:
            self._visualize_detections(color_image, depth_image, scene_objects)
            
        return scene_objects
    
    def _visualize_detections(self, color_image, depth_image, scene_objects):
        """Debug visualization of detections and depth"""
        # Draw bounding boxes and transformed coordinates
        vis_image = color_image.copy()
        for obj in scene_objects:
            x1, y1, x2, y2 = map(int, obj['box'])
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Add label with robot-frame coordinates
            label = f"{obj['label']} ({obj['score']:.2f})"
            pos = obj['position']
            coords = f"X:{pos[0]:.3f} Y:{pos[1]:.3f} Z:{pos[2]:.3f}"
            cv2.putText(vis_image, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(vis_image, coords, (x1, y1+20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Create depth visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # Show visualizations
        cv2.imshow('Detections', vis_image)
        cv2.imshow('Depth Map', depth_colormap)
        cv2.waitKey(1)
    
    def stop(self):
        """Clean up resources"""
        self.pipeline.stop()
        cv2.destroyAllWindows()