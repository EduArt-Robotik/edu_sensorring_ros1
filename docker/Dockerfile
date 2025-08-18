FROM arm64v8/ros:noetic-ros-base-focal

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install packages
USER root
RUN apt-get update \
    && apt-get install -y software-properties-common build-essential git\
    && apt-get install -y ros-noetic-tf2 ros-noetic-tf2-ros

RUN useradd -ms /bin/bash $USER\
    && sudo passwd -d $USER\
    && usermod -aG sudo user
    
USER $USER

# get the sensorring library from github
RUN mkdir -p /home/$USER/eduart_ws/ \
    && cd /home/$USER/eduart_ws \
    && git clone --depth 1 --branch v2.0.0  https://github.com/EduArt-Robotik/edu_lib_sensorring.git \
    && cd edu_lib_sensorring\
    && sudo ./cmake/install_release.bash

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY edu_sensorring_ros1 /home/$USER/ros/src/edu_sensorring_ros1

# build the sensorring ros package
RUN /bin/bash -c "echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/$USER/.bashrc\
    && echo 'source /home/$USER/ros/devel/setup.bash' >> /home/$USER/.bashrc\
    && source /opt/ros/$ROS_DISTRO/setup.bash\
    && source /home/$USER/.bashrc \
    && cd /home/$USER/ros/src\
    && catkin_init_workspace\
    && cd /home/$USER/ros\
    && catkin_make -j1 -l1 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source devel/setup.bash"

WORKDIR /home/$USER/ros

# add entry point
COPY --chown=$USER edu_sensorring_ros1/docker/entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]