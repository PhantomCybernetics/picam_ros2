# Picamera ROS2 Node

Streams hw-encoded h.264 frames as a ROS2 topic

### Build 

`docker build -f picam_ros2/Dockerfile -t "phntm/picam-ros:humble" .`

### Config example
`
/**:
  ros__parameters:
    topic_prefix: '/picam_h264/camera_'
`
