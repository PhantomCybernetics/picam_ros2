# Picamera ROS2 Node

Streams hw-encoded h.264 frames as a ROS2 topic

### Build 

`docker build -f picam_ros2/Dockerfile -t "phntm/picam-ros:humble" .`

### Config example
`yaml
/**:
  ros__parameters:
    topic_prefix: '/picam_h264/camera_'
    log_message_every_sec: 5.0
    /camera_2: # 2 is the camera location
      hflip: False
      vflip: False
      bitrate: 5000000
      framerate: 30
`
### compose.yaml:
`yaml
services:
  picam_ros:
    image: phntm/picam-ros:humble
    container_name: picam-ros
    hostname: picam-ros.local
    restart: unless-stopped
    privileged: true
    network_mode: host
    shm_size: 200m # more room for camera frames
    volumes:
      - ~/picam_ros.yaml:/ros2_ws/picam_ros_params.yaml # config goes here
      - ~/picam_ros2:/ros2_ws/src/picam_ros2
      - /tmp:/tmp
    devices:
      - /dev:/dev # cameras need this
    command:
      ros2 launch picam_ros2 picam_ros2_launch.py
      #/bin/sh -c "while sleep 1000; do :; done"
`
