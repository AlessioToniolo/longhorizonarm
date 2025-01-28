import pyrealsense2 as rs
import numpy as np
import cv2

class DepthVisualizer:
    def __init__(self, min_depth=0.1, max_depth=1.0):
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable both depth and color streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.profile = self.pipeline.start(self.config)
        
        # Getting the depth sensor's depth scale
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        # Set the depth range (in meters)
        self.min_depth = min_depth
        self.max_depth = max_depth
        
        # Create alignment object
        self.align = rs.align(rs.stream.color)

        # Create grid for point sampling
        self.create_sampling_grid(spacing=20)  # Adjust spacing to change dot density

    def create_sampling_grid(self, spacing):
        """Create a grid of points for sampling depth data"""
        height, width = 480, 640  # Standard RealSense resolution
        self.grid_y, self.grid_x = np.mgrid[0:height:spacing, 0:width:spacing]
        self.grid_y = self.grid_y.flatten()
        self.grid_x = self.grid_x.flatten()

    def process_frames(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None, None
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        return depth_image, color_image

    def get_depth_points(self, depth_image):
        """Sample depth values at grid points"""
        # Get depth values at grid points
        depth_values = depth_image[self.grid_y, self.grid_x] * self.depth_scale
        
        # Filter points within the desired depth range
        valid_points = np.logical_and(
            depth_values > self.min_depth,
            depth_values < self.max_depth
        )
        
        return (
            self.grid_y[valid_points],
            self.grid_x[valid_points],
            depth_values[valid_points]
        )

    def draw_depth_dots(self, color_image, points_y, points_x, depth_values):
        """Draw depth-indicating dots on the color image"""
        vis_image = color_image.copy()
        
        for y, x, depth in zip(points_y, points_x, depth_values):
            # Normalize depth value to 0-1 range
            normalized_depth = (depth - self.min_depth) / (self.max_depth - self.min_depth)
            normalized_depth = np.clip(normalized_depth, 0, 1)
            
            # Create color based on depth
            # Red (close) to Blue (far)
            color = (
                int(255 * normalized_depth),    # Blue
                50,                             # Green (minimal for visibility)
                int(255 * (1 - normalized_depth))  # Red
            )
            
            # Calculate dot size based on depth - larger for closer objects
            min_size = 2  # Minimum dot size (for furthest points)
            max_size = 8  # Maximum dot size (for closest points)
            size = int(min_size + (max_size - min_size) * (1 - normalized_depth))
            
            # Draw dot with border for better visibility
            cv2.circle(vis_image, (x, y), size + 1, (0, 0, 0), -1)  # Black border
            cv2.circle(vis_image, (x, y), size, color, -1)          # Colored center
            
        return vis_image

    def run(self):
        try:
            while True:
                # Get frames
                depth_image, color_image = self.process_frames()
                if depth_image is None or color_image is None:
                    continue
                
                # Get depth points from grid
                points_y, points_x, depth_values = self.get_depth_points(depth_image)
                
                # Create visualization
                vis_image = self.draw_depth_dots(
                    color_image,
                    points_y,
                    points_x,
                    depth_values
                )
                
                # Show images
                cv2.imshow('RealSense Depth Visualization', vis_image)
                
                # Break loop on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
        finally:
            # Stop streaming
            self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    # Create visualizer with custom depth range (in meters)
    visualizer = DepthVisualizer(min_depth=0.1, max_depth=1.0)
    
    # Run visualization
    visualizer.run()