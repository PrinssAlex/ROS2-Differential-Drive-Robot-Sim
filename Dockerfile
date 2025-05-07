# Base image with ROS 2 Jazzy and Gazebo
FROM osrf/ros:jazzy-desktop

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-xacro \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Create and set the working directory
WORKDIR /ros2_ws

# Copy only the necessary files into the container (use .dockerignore to exclude unnecessary files)
COPY . /ros2_ws

# ensure no stale CMake cache or install artifacts remain
RUN rm -rf /ros2_ws/build /ros2_ws/install /ros2_ws/log

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Automate sourcing of ROS 2 and workspace environments
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Expose necessary ports (e.g., for Gazebo or RViz)
EXPOSE 8080

# Default command to start a bash shell
CMD ["/bin/bash"]


