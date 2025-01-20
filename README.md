# Robot Arm for Long-Horizon

## Commands/setup

Start development
```
docker compose up -d
docker compose exec ros_dev bash
```

Now you're inside the container
`cd /ros_ws`

Create a new ROS package
```
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy urdf xacro control_msgs sensor_msgs moveit_msgs etc. etc. etc.
```
***The new package will appear in your src folder on your Mac!***

Stop the container
```
docker compose down
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