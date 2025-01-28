import numpy as np
import time
try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    print("Please install OMPL: pip install ompl")
    raise

class RobotArmPlanner:
    def __init__(self, kinematics):
        self.kinematics = kinematics
        
        # Define state space
        self.space = ob.RealVectorStateSpace(3)
        
        # Set joint limits
        bounds = ob.RealVectorBounds(3)
        # Base rotation limits
        bounds.setLow(0, 0.0)
        bounds.setHigh(0, 180.0)
        # Shoulder limits
        bounds.setLow(1, 0.0)
        bounds.setHigh(1, 180.0)
        # Elbow limits
        bounds.setLow(2, 0.0)
        bounds.setHigh(2, 180.0)
        
        self.space.setBounds(bounds)
        
        # Create simple setup
        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))
        
        # Set planner
        self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        self.ss.setPlanner(self.planner)
    
    def isStateValid(self, state):
        """Check if a state is valid (within joint limits and collision-free)"""
        # Get joint angles
        angles = [state[0], state[1], state[2]]
        
        # Basic joint limit check
        for angle in angles:
            if angle < 0 or angle > 180:
                return False
                
        # Add collision checking here if needed
        return True
    
    def plan_path(self, start_angles, goal_angles, timeout=1.0):
        """Plan a path from start to goal configuration"""
        # Set start state
        start = ob.State(self.space)
        start[0] = start_angles[0]
        start[1] = start_angles[1]
        start[2] = start_angles[2]
        
        # Set goal state
        goal = ob.State(self.space)
        goal[0] = goal_angles[0]
        goal[1] = goal_angles[1]
        goal[2] = goal_angles[2]
        
        self.ss.setStartAndGoalStates(start, goal)
        
        # Attempt to solve
        solved = self.ss.solve(timeout)
        
        if solved:
            path = []
            # Get solution path
            solution = self.ss.getSolutionPath()
            solution.interpolate()
            
            # Extract path points
            for i in range(solution.getStateCount()):
                state = solution.getState(i)
                path.append([state[0], state[1], state[2]])
                
            return path
        else:
            return None
    
    def execute_path(self, path, time_between_points=0.1):
        """Execute a planned path"""
        if not path:
            return False
            
        for angles in path:
            message = {
                "command": "move_multiple",
                "movements": [
                    {"servo_id": self.kinematics.base_servo, "position": angles[0]},
                    {"servo_id": self.kinematics.shoulder_servo, "position": angles[1]},
                    {"servo_id": self.kinematics.elbow_servo, "position": angles[2]}
                ]
            }
            self.kinematics.controller.send_command(message)
            time.sleep(time_between_points)
        return True

def main():
    from TestKinematics import TestKinematics
    
    arm = TestKinematics()
    planner = RobotArmPlanner(arm)
    
    # Test planning
    start = [90, 90, 90]  # Center position
    goal = [180, 45, 135]  # Target position
    
    path = planner.plan_path(start, goal)
    if path:
        print("Path found! Executing...")
        planner.execute_path(path)
    else:
        print("No path found")
    
    arm.close()

if __name__ == "__main__":
    main()