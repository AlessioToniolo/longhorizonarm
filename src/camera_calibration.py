import numpy as np
import cv2
import pyrealsense2 as rs
import time

class CameraCalibrator:
    def __init__(self):
        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        
        # Checkerboard settings
        self.checker_size = (9, 6)  # Number of inner corners
        self.square_size = 0.0254  # Size of squares in meters (e.g., 1 inch = 0.0254m)
        
        # Prepare object points
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        
        # Create array of checkerboard 3D coordinates
        self.objp = np.zeros((self.checker_size[0] * self.checker_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.checker_size[0], 0:self.checker_size[1]].T.reshape(-1,2)
        self.objp *= self.square_size
        
        # Capture settings
        self.last_capture_time = 0
        self.capture_delay = 2  # Seconds between captures
        self.required_captures = 20
        
    def capture_images(self):
        """Capture images for calibration"""
        print("\nCamera Calibration Instructions:")
        print("1. Hold the checkerboard in different positions and orientations")
        print("2. Make sure the entire checkerboard is visible")
        print("3. The script will automatically capture when checkerboard is detected")
        print("4. Try to cover different angles and distances")
        print("5. Press 'q' to quit when done\n")
        
        while len(self.img_points) < self.required_captures:
            # Get frame
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
                
            # Convert to numpy array
            frame = np.asanyarray(color_frame.get_data())
            display_frame = frame.copy()
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Find checkerboard corners
            current_time = time.time()
            ret, corners = cv2.findChessboardCorners(gray, self.checker_size, None)
            
            if ret:
                # Draw corners
                cv2.drawChessboardCorners(display_frame, self.checker_size, corners, ret)
                
                # If enough time has passed, save the points
                if current_time - self.last_capture_time > self.capture_delay:
                    # Refine corner positions
                    corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    
                    self.obj_points.append(self.objp)
                    self.img_points.append(corners)
                    
                    self.last_capture_time = current_time
                    print(f"Captured image {len(self.img_points)}/{self.required_captures}")
            
            # Display info
            cv2.putText(display_frame, f"Captures: {len(self.img_points)}/{self.required_captures}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show frame
            cv2.imshow('Calibration', display_frame)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                if len(self.img_points) < 10:
                    print("Warning: Not enough captures for accurate calibration")
                break
    
    def calibrate(self):
        """Perform calibration and save results"""
        print("\nPerforming calibration...")
        if len(self.img_points) < 10:
            print("Error: Not enough captures for calibration")
            return None
            
        # Get frame size
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())
        img_size = frame.shape[:2]
        
        # Perform calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, img_size[::-1], None, None)
        
        # Calculate reprojection error
        mean_error = 0
        for i in range(len(self.obj_points)):
            imgpoints2, _ = cv2.projectPoints(self.obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.img_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print(f"\nCalibration complete!")
        print(f"Average reprojection error: {mean_error/len(self.obj_points)}")
        
        # Save calibration
        calibration_data = {
            'camera_matrix': mtx.tolist(),
            'dist_coeffs': dist.tolist(),
            'image_size': img_size
        }
        
        np.save('camera_calibration.npy', calibration_data)
        print("\nCalibration saved to 'camera_calibration.npy'")
        print("\nCamera Matrix:")
        print(mtx)
        print("\nDistortion Coefficients:")
        print(dist)
        
        return calibration_data
    
    def test_calibration(self, calibration_data):
        """Test calibration by showing undistorted feed"""
        print("\nShowing undistorted feed (press 'q' to quit)")
        mtx = np.array(calibration_data['camera_matrix'])
        dist = np.array(calibration_data['dist_coeffs'])
        
        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
                
            frame = np.asanyarray(color_frame.get_data())
            
            # Undistort
            undist = cv2.undistort(frame, mtx, dist)
            
            # Show original and undistorted
            cv2.imshow('Original', frame)
            cv2.imshow('Undistorted', undist)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    def cleanup(self):
        """Clean up resources"""
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main():
    calibrator = CameraCalibrator()
    try:
        # Capture calibration images
        calibrator.capture_images()
        
        # Perform calibration
        calibration_data = calibrator.calibrate()
        
        if calibration_data:
            # Test calibration
            calibrator.test_calibration(calibration_data)
    
    finally:
        calibrator.cleanup()

if __name__ == "__main__":
    main()