FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common \
                       curl \
                       git

# ROS1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && \
    apt-get install -y \
      ros-noetic-desktop \
      ros-noetic-pcl-ros \
      ros-noetic-grid-map \
      ros-noetic-jsk-recognition-msgs

RUN git clone https://github.com/kamibukuro5656/MapCleaner_Unofficial.git /MapCleaner/src && \
    cd /MapCleaner/src && \
    git checkout 426581c066f67da5c645445266f8df53101597df && \
    cd /MapCleaner && \
    . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release

COPY ./ENTRYPOINT.sh /ENTRYPOINT.sh
RUN chmod +x /ENTRYPOINT.sh
ENTRYPOINT ["/ENTRYPOINT.sh"]