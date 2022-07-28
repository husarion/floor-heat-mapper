ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y  ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-rqt-image-view ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-visualization-msgs \
    libopencv-dev libboost-python-dev \
    && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
COPY ros_entrypoint.sh /
ENTRYPOINT ["../ros_entrypoint.sh"]
RUN chmod +x /ros_entrypoint.sh

WORKDIR /app
COPY src/vision_opencv /app/src/vision_opencv
RUN cd /app && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select cv_bridge
COPY src/floor_heat_mapper /app/src/floor_heat_mapper

RUN cd /app && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select floor_heat_mapper