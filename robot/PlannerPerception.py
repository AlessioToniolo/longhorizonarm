import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from RobotArmPlanner import RobotArmPlanner
import time

class PlannerPerception:
    def __init__(self):
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(self.config)
        
        # Initialize point cloud processor
        self.pc = rs.pointcloud()
        
        # Initialize workspace boundaries (in meters)
        self.workspace_bounds = {
            'x': (-0.5, 0.5),   # meters
            'y': (-0.5, 0.5),   # meters
            'z': (0.0, 0.8)     # meters
        }
        
        # Obstacle list
        self.obstacles = []
        
    def capture_point_cloud(self):
        """Capture and process point cloud from RealSense"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Create point cloud
        points = self.pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices())
        
        # Convert to Open3D format
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vtx.reshape(-1, 3))
        
        return pcd
    
    def segment_obstacles(self, pcd):
        """Segment obstacles from point cloud"""
        # Remove points outside workspace
        points = np.asarray(pcd.points)
        mask = (points[:, 0] >= self.workspace_bounds['x'][0]) & \
               (points[:, 0] <= self.workspace_bounds['x'][1]) & \
               (points[:, 1] >= self.workspace_bounds['y'][0]) & \
               (points[:, 1] <= self.workspace_bounds['y'][1]) & \
               (points[:, 2] >= self.workspace_bounds['z'][0]) & \
               (points[:, 2] <= self.workspace_bounds['z'][1])
        
        pcd = pcd.select_by_index(np.where(mask)[0])
        
        # Segment obstacles using DBSCAN clustering
        labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=50))
        
        # Extract obstacle clusters
        obstacles = []
        for label in set(labels):
            if label == -1:  # Noise points
                continue
            cluster = pcd.select_by_index(np.where(labels == label)[0])
            obstacles.append(cluster)
        
        return obstacles
    
    def update_planner_obstacles(self, planner, obstacles):
        """Update OMPL planner with detected obstacles"""
        def is_state_valid(state):
            # Convert joint angles to end effector position
            angles = [state[0], state[1], state[2]]
            
            # Check joint limits
            if not planner.isStateValid(state):
                return False
            
            # Check collision with obstacles
            for obstacle in obstacles:
                # Convert obstacle points to robot frame
                obs_points = np.asarray(obstacle.points)
                
                # Simple sphere-based collision checking
                # You would need to implement forward kinematics here
                # to check actual robot arm collision
                
                return True  # Placeholder
            
            return True
        
        # Update validity checker
        planner.ss.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))
    
    def run_perception_loop(self, planner):
        """Main perception loop"""
        try:
            while True:
                # Capture and process point cloud
                pcd = self.capture_point_cloud()
                
                # Segment obstacles
                obstacles = self.segment_obstacles(pcd)
                
                # Update planner
                self.update_planner_obstacles(planner, obstacles)
                
                # Visualize (optional)
                self.visualize_scene(pcd, obstacles)
                
                time.sleep(0.1)  # Run at 10Hz
                
        finally:
            self.pipeline.stop()
    
    def visualize_scene(self, pcd, obstacles):
        """Visualize point cloud and detected obstacles"""
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        
        # Add point cloud
        vis.add_geometry(pcd)
        
        # Add obstacles with different colors
        for i, obstacle in enumerate(obstacles):
            obstacle.paint_uniform_color([1.0, 0.0, 0.0])  # Red for obstacles
            vis.add_geometry(obstacle)
        
        vis.run()
        vis.destroy_window()

def main():
    from TestKinematics import TestKinematics
    
    arm = TestKinematics()
    planner = RobotArmPlanner(arm)
    perception = PlannerPerception()
    
    try:
        perception.run_perception_loop(planner)
    finally:
        arm.close()

if __name__ == "__main__":
    main()