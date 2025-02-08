# Raspberry Pi Camera ROS2 Node

Streams hardware or software encoded H.264 frames as ROS2 topics, optionally also allows to stream YUV420/Mono8 or BGR8 uncompressed frames as an Image topic.

Using libcamera to capture frames, and v4l2 with BCM2711 or libav for CPU-bases encoding. DMA heaps are used for fast frame access.

This node allows to calibrate the camera via ROS2 service calls, then streams calibration data as a CameraInfo topic.

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
docker build -f Dockerfile -t phntm/picam-ros2:$ROS_DISTRO \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  .
```

### Configure the Node
The following is an example config file (~/picam_ros2_params.yaml)
```yaml
/**:
  ros__parameters:
    topic_prefix: '/picam_ros2/camera_'
    log_message_every_sec: 5.0
    log_scroll: False
    calibration_frames_needed: 10
    calibration_square_size_m: 0.0175 # set this to your actual calibration square dimension!
    calibration_pattern_size: [ 9, 6 ] # set this to your calibration chessboard size!
    calibration_files: '/calibration' # calibration files saved here

    /camera_2: # 2 is the camera location
      frame_id: 'pi_camera_optical_frame'

      enabled: True
      width: 1920
      height: 1080
      hflip: False
      vflip: False

      hw_encoder: True
      bitrate: 3000000
      compression: 30
      framerate: 30

      publish_h264: True
      publish_image: False
      image_output_format: yuv420
      publish_info: True

      enable_calibration: True

      buffer_count: 4
      
      # exposure_time_ns: 30000
      # analog_gain: 2.0
      contrast: 1.3
      ae_enable: True
      awb_enable: True
      awb_mode: 0
      # awb_locked: False
```

### Add Service to your compose.yaml:
```yaml
services:
  picam_ros2:
    image: phntm/picam-ros2:humble
    container_name: picam-ros2
    hostname: picam-ros2.local
    restart: unless-stopped
    privileged: true
    network_mode: host
    ipc: host # phntm bridge needs this to see other local containers
    # cpuset: '3' # consider restricting to a single CPU core
    # shm_size: 200m # more room for camera frames
    volumes:
      - ~/picam_ros2_params.yaml:/ros2_ws/picam_ros2_params.yaml # config goes here
      - /tmp:/tmp
      - ~/picam_ros2_calibration:/calibration # calibration files are stored here
    devices:
      - /dev:/dev # cameras need this
    command:
      ros2 launch picam_ros2 picam_ros2_launch.py
```

### Launch
```bash
docker compose up picam_ros2
```
