import pyrealsense2 as rs
import numpy as np

class RealSenseInterface:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable both streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.profile = self.pipeline.start(self.config)
        
        # Get depth sensor
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

    def get_frames(self):
        """Get both color and depth frames"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        return color_frame, depth_frame

    def get_depth_scale(self):
        """Return depth scale from sensor"""
        return self.depth_scale

    def stop(self):
        self.pipeline.stop()