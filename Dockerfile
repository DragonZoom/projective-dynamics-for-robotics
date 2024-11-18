FROM ros:jazzy-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libglew-dev \
    libglfw3-dev \
    libglm-dev \
    libglu1-mesa-dev \
    mesa-utils \
    x11-apps \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxrandr2 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs \
    ros-jazzy-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Set up the workspace
WORKDIR /workspace
COPY . /workspace

# Clean up workspace by removing existing build artifacts
RUN rm -rf /workspace/build /workspace/install /workspace/log

# Install package dependencies
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Source ROS environment and build the package using colcon
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

CMD ["/bin/bash"]
