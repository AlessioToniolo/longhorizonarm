import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from transformers import AutoModelForObjectDetection, DetrImageProcessor

class ScenePerception:
    def __init__(self, enable_visualization=False):
        # Initialize RealSense streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

        # Initialize VLM model and processor
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.model = AutoModelForObjectDetection.from_pretrained("facebook/detr-resnet-50").to(self.device)

        # Camera intrinsics
        self.fx = 615.77
        self.fy = 615.84
        self.cx = 326.47
        self.cy = 240.87

        self.enable_visualization = enable_visualization

    def get_scene_objects(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return []

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Run object detection
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb_image, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = self.model(**inputs)
        target_sizes = torch.tensor([rgb_image.shape[:2]]).to(self.device)
        results = self.processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.5)[0]

        objects = []
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            x1, y1, x2, y2 = map(int, box.cpu().numpy())
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            depth = depth_image[center_y, center_x] * self.depth_scale
            X = (center_x - self.cx) * depth / self.fx
            Y = (center_y - self.cy) * depth / self.fy
            Z = depth
            objects.append({
                'label': self.model.config.id2label[label.item()],
                'score': score.cpu().numpy(),
                'position': (X, Y, Z),
                'box': [x1, y1, x2, y2]
            })

        if self.enable_visualization:
            self._visualize(color_image, depth_image, objects)
        return objects

    def _visualize(self, color_image, depth_image, objects):
        vis = color_image.copy()
        for obj in objects:
            x1, y1, x2, y2 = obj['box']
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{obj['label']} ({obj['score']:.2f})"
            pos = obj['position']
            coords = f"X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}"
            cv2.putText(vis, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(vis, coords, (x1, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        depth_vis = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow("Detections", vis)
        cv2.imshow("Depth Map", depth_vis)
        cv2.waitKey(1)

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
