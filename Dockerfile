ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO
ARG ARCH=aarch64

RUN echo "Building docker image with ROS_DISTRO=$ROS_DISTRO, ARCH=$ARCH"

RUN apt-get update -y --fix-missing
RUN apt-get install -y ssh \
                       vim mc \
                       iputils-ping net-tools iproute2 curl \
                       pip

# aiorc neeed pip update or fails on cffi version inconsistency
RUN pip install --upgrade pip

# aiortc dev dependencies
RUN apt-get update -y --fix-missing
RUN apt-get install -y libavdevice-dev libavfilter-dev libopus-dev libvpx-dev pkg-config

# gazebo
# RUN apt install -y ros-$ROS_DISTRO-ros-gz

RUN pip install setuptools==58.2.0

# raspi extras
RUN apt-get install -y libraspberrypi0 libraspberrypi-dev libraspberrypi-bin

# video stuffs
RUN apt-get install -y v4l-utils ffmpeg

#libcamera deps
# RUN pip3 install --user meson
# RUN pip3 install --user --upgrade meson
# RUN apt-get install -y ninja-build pkg-config
# RUN apt-get install -y libyaml-dev python3-yaml python3-ply python3-jinja2
# RUN apt-get install -y libudev-dev
# RUN apt-get install -y libevent-dev
# RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-de

RUN pip3 install --user meson
RUN apt install -y python3-jinja2
RUN apt install -y libboost-dev
RUN apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
RUN apt install -y python3-yaml python3-ply
RUN apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
RUN apt install -y ninja-build pkg-config

# RUN echo "export PATH=\$PATH:/root/.local/bin" >> /root/.bashrc
ENV PATH=$PATH":/root/.local/bin"

# init workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p $ROS_WS/src

# kms++ from source (for picamera2) \
# RUN apt-get install -y libdrm-common libdrm-dev
# WORKDIR $ROS_WS
# RUN git clone https://github.com/tomba/kmsxx.git
# WORKDIR $ROS_WS/kmsxx
# RUN git submodule update --init
# ENV PYTHONPATH=$PYTHONPATH":/ros2_ws/kmsxx/build/py"
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH":/usr/local/lib/"$ARCH"-linux-gnu"
# RUN /root/.local/bin/meson build
# RUN ninja -C build install

RUN apt-get install -y libcap-dev

# libcamera (makes its python bidings for picamera2)
WORKDIR $ROS_WS

RUN apt-get install -y udev

# libcamera from raspi fork
RUN git clone https://github.com/raspberrypi/libcamera.git
WORKDIR $ROS_WS/libcamera
# RUN /root/.local/bin/meson setup build -D pycamera=enabled -D v4l2=True --reconfigure
RUN /root/.local/bin/meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
RUN ninja -C build install

# ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH":/usr/local/lib/aarch64-linux-gnu/"

# libcamera v0.2.0 from raspi fork works with picamera2==0.3.17 (current)
# works on pi5 w bookworm (sw only encoding)

# RUN git clone https://github.com/raspberrypi/libcamera.git
# WORKDIR $ROS_WS/libcamera
# RUN /root/.local/bin/meson setup build -D pycamera=enabled -D v4l2=True --reconfigure
# RUN ninja -C build install
# WORKDIR $ROS_WS
# RUN git clone https://github.com/raspberrypi/picamera2.git
# RUN pip install -e /ros2_ws/picamera2

# ENV PYTHONPATH=$PYTHONPATH":/ros2_ws/libcamera/build/src/py"

# needed by reload-devies.sh (reloads docker devices after the container has been created)


# fix numpy version to >= 1.25.2
# RUN pip install numpy --force-reinstall

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