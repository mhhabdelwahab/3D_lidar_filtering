# Start from official ROS Noetic (for Ubuntu 20.04) image
FROM ros:noetic

# 1) Declare an ARG for DEBIAN_FRONTEND.
ARG DEBIAN_FRONTEND=noninteractive

# 2) Install packages without interactive prompts.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        ros-noetic-pcl-ros \
        ros-noetic-pcl-conversions \
        ros-noetic-rviz \
        nano\
    && rm -rf /var/lib/apt/lists/*

# 3. Create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# 4. Copy your package into container
COPY . /catkin_ws/src/lidar_filtering_assignment

# 5. Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 6. Environment setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# 7. Default command
CMD ["/bin/bash"]