import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
import cv2
from perception.camera.realsense_interface import RealSenseInterface

class DepthVisualizer:
    def __init__(self, min_depth=0.1, max_depth=1.0, stream=None):
        # Set the depth range (in meters)
        self.min_depth = min_depth
        self.max_depth = max_depth
        if stream == None:
            self.realsense = RealSenseInterface()
        else:
            self.realsense = stream
        self.depth_scale = self.realsense.get_depth_scale()

    def process_frames(self):
        """Process frames from shared RealSense interface"""
        color_frame, depth_frame = self.realsense.get_frames()
        if color_frame is not None and depth_frame is not None:
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            return depth_image, color_image
        return None, None

    def get_depth_points(self, depth_image):
        """Create sampling grid and get depth values"""
        height, width = depth_image.shape
        step = 20  # Sampling frequency
        
        y_indices, x_indices = np.mgrid[0:height:step, 0:width:step]
        points_y = y_indices.flatten()
        points_x = x_indices.flatten()
        
        depth_values = depth_image[points_y, points_x] * self.depth_scale
        
        return points_y, points_x, depth_values

    def draw_depth_dots(self, color_image, points_y, points_x, depth_values):
        """Draw depth visualization dots on color image"""
        vis_image = color_image.copy()
        
        for i in range(len(points_y)):
            if self.min_depth < depth_values[i] < self.max_depth:
                # Normalize depth and calculate dot size
                normalized_depth = (depth_values[i] - self.min_depth) / (self.max_depth - self.min_depth)
                dot_size = int(8 * (1 - normalized_depth))  # Size ranges from 1 to 8 pixels
                
                # Color mapping (red=close, blue=far)
                color = tuple(int(c) for c in [255 * (1-normalized_depth), 0, 255 * normalized_depth])
                
                cv2.circle(vis_image, (points_x[i], points_y[i]), max(1, dot_size), color, -1)
        
        return vis_image


    def stop(self):
        """Stop the RealSense interface"""
        self.realsense.stop()

def main():
    depth_vis = DepthVisualizer(min_depth=0.05, max_depth=1.0)
    try:
        while True:
            depth_image, color_image = depth_vis.process_frames()
            if depth_image is not None and color_image is not None:
                points_y, points_x, depth_values = depth_vis.get_depth_points(depth_image)
                vis_image = depth_vis.draw_depth_dots(color_image, points_y, points_x, depth_values)
                cv2.imshow('Depth Visualization', vis_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        depth_vis.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()