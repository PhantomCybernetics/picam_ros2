# PiCamera ROS2 Node

Streams hw-encoded H.264 frames as ROS2 topics

## Install

### Install Docker, Docker Build & Docker Compose

E.g. on Debian/Ubuntu follow [these instructions](https://docs.docker.com/engine/install/debian/). Then add the current user to the docker group:
```bash
sudo usermod -aG docker ${USER}
# log out & back in
```

### Build the Docker Image
```bash
cd ~
git clone git@github.com:PhantomCybernetics/picam_ros2.git picam_ros2
cd picam_ros2
ROS_DISTRO=humble; \
docker build -f Dockerfile -t phntm/picam-ros:$ROS_DISTRO \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  .
```

### Configure the Node
The following is an example config file (~/picam_ros.yaml)
```yaml
/**:
  ros__parameters:
    topic_prefix: '/picam_h264/camera_'
    log_message_every_sec: 5.0
    /camera_2: # 2 is the camera location
      hflip: False
      vflip: False
      bitrate: 5000000
      framerate: 30
```

### Add Service to your compose.yaml:
```yaml
services:
  picam_ros:
    image: phntm/picam-ros:humble
    container_name: picam-ros
    hostname: picam-ros.local
    restart: unless-stopped
    privileged: true
    # cpuset: '3' # consider restricting to a single CPU core
    network_mode: host
    ipc: host # phntm bridge needs this to see other local containers
    shm_size: 200m # more room for camera frames
    volumes:
      - ~/picam_ros.yaml:/ros2_ws/picam_ros_params.yaml # config goes here
      - /tmp:/tmp
    devices:
      - /dev:/dev # cameras need this
    command:
      ros2 launch picam_ros2 picam_ros2_launch.py
```

### Launch
```bash
docker compose up picam_ros
```