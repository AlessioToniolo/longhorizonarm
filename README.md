# Robot Arm for Long-Horizon

## Commands/setup

Start development
```
docker compose up -d
docker compose exec ros_dev bash
```

Now you're inside the container
`cd /ros_ws`
`cd src`

Create a new ROS package
```
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy urdf xacro control_msgs sensor_msgs moveit_msgs etc. etc. etc.
```
***The new package will appear in your src folder on your Mac!***

Running nodes and launch files
```
ros2 launch arm_bringup main_arm.launch.py
```
***You can launch all packages/nodes/launch files from this main launch file***

Stop the container
```
docker compose down
```

Running stuff
```
cd /ros_ws
colcon build
source install/setup.bash
ros2 run arduino_bridge blink
```
Then in another terminal:
```
bashCopyros2 topic pub /led_state std_msgs/Bool "data: true"
```

## What about python dependencies??
```
RUN pip3 install torch numpy scipy etc. etc. etc.
```
Place before `RUN echo` line in Dockerfile

## Sample project struct.
```
ros_project/
└── src/
    ├── arm_description/        # Package for robot URDF, meshes, etc.
    ├── arm_controllers/        # Package for control logic
    ├── arm_moveit_config/      # Package for MoveIt configuration
    └── arm_interface/          # Package for high-level robot commands
```

## ROS info
topics (pub/sub), services (request/response), and actions (long-running tasks)

Ex:
```
arm_controllers package:
- joint_controller_node
  - Subscribes to: /joint_commands
  - Publishes to: /joint_states
  - Provides service: /set_joint_position

arm_interface package:
- arm_interface_node
  - Publishes to: /joint_commands
  - Subscribes to: /joint_states
  - Calls service: /set_joint_position

Action would be MoveToXY
  ```

## codebase standards
- separate pub/sub between packages
- One package per major functionality (controllers, hardware interface, planning)
- ==Keep launch files separate from nodes==
- one node a file
- class based nodes
- separate config files for parameters
- separate dev and prod dockerfiles **TODO**
- Mount source code as volumes during development
- Use .dockerignore for unnecessary files
- Use actions for long-running tasks (movement)
- Use services for quick queries
- Use topics for continuous data streams
- Test nodes in isolation

## final tip
github copilot for terminal commands