ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO
ARG ARCH=aarch64

RUN echo "Building docker image with ROS_DISTRO=$ROS_DISTRO, ARCH=$ARCH"

RUN apt-get update -y --fix-missing
RUN apt-get install -y ssh \
                       vim mc \
                       iputils-ping net-tools iproute2 curl

RUN apt-get update -y --fix-missing
RUN apt-get install -y libavdevice-dev libavfilter-dev libopus-dev libvpx-dev pkg-config

RUN apt-get install -y python3-setuptools

# raspi extras
RUN apt-get install -y libraspberrypi0 libraspberrypi-dev libraspberrypi-bin

# video stuffs
RUN apt-get install -y v4l-utils ffmpeg
RUN apt install -y python3-jinja2
RUN apt install -y libboost-dev
RUN apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
RUN apt install -y python3-yaml python3-ply
RUN apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
RUN apt install -y --no-install-recommends libopencv-dev
RUN apt install -y libjsoncpp-dev
RUN apt install -y libyaml-cpp-dev
RUN apt install -y build-essential
RUN apt install -y ninja-build pkg-config
RUN apt install -y pip
RUN apt-get install -y libcap-dev
RUN apt-get install -y udev

# init workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p $ROS_WS/src

RUN apt-get install -y python3-venv
RUN mkdir -p /root/ros2_py_venv
RUN python3 -m venv /root/ros2_py_venv
RUN . /root/ros2_py_venv/bin/activate && \
    pip install meson && \
    deactivate

ENV PATH=$PATH":/root/ros2_py_venv/bin"
ENV LD_LIBRARY_PATH="/usr/local/lib/"$ARCH"-linux-gnu"

# libcamera from raspi fork
WORKDIR $ROS_WS
RUN git clone https://github.com/raspberrypi/libcamera.git
WORKDIR $ROS_WS/libcamera
RUN /root/ros2_py_venv/bin/meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
RUN ninja -C build install

# generate entrypoint script
RUN echo '#!/bin/bash \n \
set -e \n \
\n \
# setup ros environment \n \
source "/opt/ros/'$ROS_DISTRO'/setup.bash" \n \
test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash" \n \
\n \
exec "$@"' > /ros_entrypoint.sh

RUN chmod a+x /ros_entrypoint.sh

# source underlay on every login
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root/.bashrc
RUN echo 'test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash"' >> /root/.bashrc

WORKDIR $ROS_WS

# install picam_ros2
COPY ./ $ROS_WS/src/picam_ros2
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
     rosdep install -i --from-path src/picam_ros2 --rosdistro $ROS_DISTRO -y && \
     colcon build --packages-select picam_ros2

# pimp up prompt with hostame and color
RUN echo "PS1='\${debian_chroot:+(\$debian_chroot)}\\[\\033[01;35m\\]\\u@\\h\\[\\033[00m\\] \\[\\033[01;34m\\]\\w\\[\\033[00m\\] '"  >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]