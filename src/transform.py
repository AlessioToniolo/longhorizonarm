import numpy as np
import cv2
import pupil_apriltags as apriltag

class RobotTransform:
    def __init__(self):
        # AprilTag detector setup
        self.tag_detector = apriltag.Detector(families='tag36h11')
        
        # Camera intrinsics (you'll need to calibrate these)
        self.camera_matrix = np.array([
            [615.7725830078125, 0.0, 326.4713134765625],
            [0.0, 615.8447875976562, 240.8659210205078],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((4,1))  # Assuming no distortion for simplicity
        
        # Transform matrices
        self.T_cam_tag = None  # Camera to AprilTag transform
        self.T_tag_robot = None  # AprilTag to robot base transform
        self.T_cam_robot = None  # Final camera to robot transform
        
        # Tag size in meters
        self.tag_size = 0.05  # Adjust based on your tag size

    def detect_apriltag(self, image):
        """Detect AprilTag and estimate its pose"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = self.tag_detector.detect(gray)
        
        if not results:
            return None
            
        # Get the first detected tag
        tag = results[0]
        
        # Get rotation and translation vectors
        corners = tag.corners.reshape(4, 2)
        corners = np.array(corners, dtype=np.float32)
        
        # Corresponding 3D points of tag corners
        obj_pts = np.array([
            [-self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2,  self.tag_size/2, 0],
            [-self.tag_size/2,  self.tag_size/2, 0]
        ])
        
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(obj_pts, corners, self.camera_matrix, self.dist_coeffs)
        
        if not success:
            return None
            
        # Convert rotation vector to matrix
        R_cam_tag, _ = cv2.Rodrigues(rvec)
        
        # Build transformation matrix
        self.T_cam_tag = np.eye(4)
        self.T_cam_tag[:3, :3] = R_cam_tag
        self.T_cam_tag[:3, 3] = tvec.flatten()
        
        return self.T_cam_tag

    def set_robot_to_tag_transform(self, robot_pos, robot_orientation):
        """Set the transform from tag to robot base
        Args:
            robot_pos: (x, y, z) position of robot base relative to tag
            robot_orientation: (roll, pitch, yaw) orientation of robot base
        """
        roll, pitch, yaw = robot_orientation
        
        # Create rotation matrix from Euler angles
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        R_tag_robot = Rz @ Ry @ Rx
        
        # Build transformation matrix
        self.T_tag_robot = np.eye(4)
        self.T_tag_robot[:3, :3] = R_tag_robot
        self.T_tag_robot[:3, 3] = robot_pos
        
        # Update camera to robot transform if we have both transforms
        if self.T_cam_tag is not None:
            self.T_cam_robot = self.T_tag_robot @ np.linalg.inv(self.T_cam_tag)

    def transform_point(self, point_camera):
        """Transform a point from camera frame to robot frame
        Args:
            point_camera: (x, y, z) point in camera frame
        Returns:
            (x, y, z) point in robot frame
        """
        if self.T_cam_robot is None:
            return None
            
        # Convert point to homogeneous coordinates
        point_hom = np.array([*point_camera, 1])
        
        # Transform point
        point_robot_hom = self.T_cam_robot @ point_hom
        
        # Convert back to 3D coordinates
        point_robot = point_robot_hom[:3] / point_robot_hom[3]
        
        return point_robot