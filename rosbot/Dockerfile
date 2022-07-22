ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    # OpenCV dependencies
    build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    libcanberra-gtk-module libcanberra-gtk3-module

# Clone, build and install OpenCV
RUN git clone https://github.com/opencv/opencv.git && \
    cd /opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j"$(nproc)" && \
    make install && \
    rm -rf /opencv

RUN apt-get update && \
    apt-get install -y  ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-rqt-image-view \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
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