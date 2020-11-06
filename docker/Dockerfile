# "jacinto_ros_perception" should be populated under $WORK_DIR/catkin_ws/src
# cd $WORK_DIR
# docker build -t "<label>" -f ./catkin_ws/src/jacinto_ros_perception/docker/Dockerfile .

FROM arm64v8/ubuntu:18.04 as melodic_ros
ARG DEBIAN_FRONTEND=noninteractive

## settings for tivision_apps.so: install OpenGL, libdevil-dev
RUN apt-get update && \
    apt-get install -y \
    libdevil-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev && \
    rm -rf /var/lib/apt/lists/*

## install utils
RUN apt-get update && \
    apt-get install -y \
    wget curl git vim tmux net-tools iputils-ping && \
    rm -rf /var/lib/apt/lists/*

## install ROS
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=melodic
ARG PROJECT_HOME=j7ros_home
ENV WORK_DIR=/root/$PROJECT_HOME
ENV CATKIN_WS=$WORK_DIR/catkin_ws

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# install ros-core packages
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    ros-melodic-ros-core=1.4.1-0* && \
    rm -rf /var/lib/apt/lists/*

## install ROS apps
COPY ./catkin_ws/src $CATKIN_WS/src

## install ROS dependency and remove ROS apps. ROS apps will be volume-mapped at "docker run"
RUN apt-get update && \
    rosdep install --from-paths $CATKIN_WS/src/ --ignore-src -r -y && \
    rm -rf $CATKIN_WS/src && \
    rm -rf /var/lib/apt/lists/*

## .profile and .bashrc
WORKDIR /root
RUN echo "if [ -n \"$BASH_VERSION\" ]; then"     >  .profile && \
    echo "    # include .bashrc if it exists"    >> .profile && \
    echo "    if [ -f \"$HOME/.bashrc\" ]; then" >> .profile && \
    echo "        . \"$HOME/.bashrc\""           >> .profile && \
    echo "    fi"                                >> .profile && \
    echo "fi"                                    >> .profile && \
    echo "#!/bin/bash"                           >  .bashrc  && \
    echo "export PS1=\"${debian_chroot:+($debian_chroot)}\u@j7-docker:\w\$ \"" >> .bashrc

#============================================================
FROM scratch
COPY --from=melodic_ros / /

# setup environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=melodic
ARG PROJECT_HOME=j7ros_home
ENV WORK_DIR=/root/$PROJECT_HOME
ENV CATKIN_WS=$WORK_DIR/catkin_ws

# workdir for ROS apps
WORKDIR $CATKIN_WS

# setup entrypoint
ARG GIT_REPO=jacinto_ros_perception
ADD ./catkin_ws/src/$GIT_REPO/docker/entrypoint.sh /root/
ENTRYPOINT ["/root/entrypoint.sh"]
