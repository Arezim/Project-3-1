# Dockerfile
FROM moveit/moveit:noetic-release

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    git \
    ros-noetic-moveit \
 && rm -rf /var/lib/apt/lists/*

# download the repository
WORKDIR /root
RUN git clone https://github.com/Arezim/Project-3-1.git

# Path to the workspace
ENV WS=/root/Project-3-1/motion-planning-1/ws
WORKDIR ${WS}

# Initialization and build of catkin
RUN rm -rf build devel .catkin_tools \
 && catkin init \
 && catkin config --extend /opt/ros/noetic \
 && chmod +x src/iiwa_dynamic_demo/scripts/*.py \
 && catkin build

# Autoload environment in every interactive bash session
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc \
 && echo "source ${WS}/devel/setup.bash" >> /etc/bash.bashrc

ENV DISPLAY=host.docker.internal:0.0
ENV DISABLE_ROS1_EOL_WARNINGS=1

# Default working directory and command
WORKDIR ${WS}
CMD ["bash"]
