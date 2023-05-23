# syntax = docker/dockerfile:1.0-experimental

FROM osrf/ros:melodic-desktop-bionic

# install build dependacies
RUN apt-get update && \
    apt-get install -y apt-utils make python-catkin-tools qt5-default ros-melodic-pcl-ros nano

# Create third party folder
ARG THIRD_PARTY_PATH=/root/third_party
RUN mkdir $THIRD_PARTY_PATH
WORKDIR $THIRD_PARTY_PATH

RUN rosdep update && \
    rosdep install -y \
    --from-paths . \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    --as-root=apt:false
RUN rm -rf /var/lib/apt/lists/*

# install third party libs
RUN git clone https://github.com/fada-catec/piloting-mavsdk && \
    cd piloting-mavsdk && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j6 && \
    make install

RUN git clone https://github.com/fada-catec/qslog.git && \
    cd qslog && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j6 && \
    make install

# setup workspace
ARG PILOTING_WS=/root/piloting_ws
RUN mkdir -p $PILOTING_WS/src
WORKDIR $PILOTING_WS/src
RUN git clone -b docker_head https://github.com/fada-catec/piloting_grcs.git

WORKDIR $PILOTING_WS
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO'
RUN cd $PILOTING_WS/build && make install

WORKDIR "/root"
RUN rm -r $PILOTING_WS
RUN rm -r $THIRD_PARTY_PATH

# setup entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# set environment variables
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
