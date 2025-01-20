
# Use ROS Humble as base (good for M1 Mac as it supports ARM64)
FROM ros:humble

# Install necessary development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /ros_ws

# Python dependencies
RUN pip3 install torch numpy scipy

# Source ROS environment in bashrc so it's available when we log into container
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Keep container running (instead of exiting immediately)
CMD ["bash"]