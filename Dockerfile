ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

SHELL ["/bin/bash", "-c"]
RUN apt update && \
    apt-get install -y  ros-$ROS_DISTRO-nav-msgs\
    && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
COPY ros_entrypoint.sh /
ENTRYPOINT ["../ros_entrypoint.sh"]
RUN chmod +x /ros_entrypoint.sh

WORKDIR /app
COPY src/ /app/src/
RUN cd /app && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build