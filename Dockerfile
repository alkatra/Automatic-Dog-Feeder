# Use an official ROS 2 base image
FROM osrf/ros:iron-desktop

# Install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Set up the environment
ENV AMENT_PREFIX_PATH=/opt/ros/iron
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Install rosbridge_suite
RUN apt-get update && apt-get install -y \
    ros-iron-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Copy your ROS 2 workspace into the Docker image
COPY ./adf_ws /root/adf_ws

# Handle ROS dependencies
RUN rosdep update \
    && rosdep install --from-paths /root/adf_ws/src --ignore-src --rosdistro iron -y

# Build the ROS 2 workspace
RUN . /opt/ros/iron/setup.sh && \
    cd /root/adf_ws && \
    colcon build --packages-select adf_package

# Source the workspace
RUN echo "source /root/adf_ws/install/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR /root/

# Define the entry point script to start rosbridge server and ROS 2 launch file
CMD ["/bin/bash", "-c", "source /opt/ros/iron/setup.bash && source /root/adf_ws/install/setup.bash && ros2 launch adf_package adf_package_launch.py & ros2 run rosbridge_server rosbridge_websocket"]
