# Raspberry Pi CSI Camera ROS2 Node

Streams hardware or software encoded H.264 frames as ROS2 topics, optionally also allows to stream YUV420/Mono8 or BGR8 uncompressed frames as an Image topic.

Using libcamera to capture frames, and v4l2 with BCM2711 or libav for CPU-bases encoding. DMA heaps are used for fast frame access.

This node allows to calibrate the camera via ROS2 service calls, then streams calibration data as a CameraInfo topic.

This package was designed to work with [Phantom Bridge](https://docs.phntm.io/bridge) and to provide fast hardware-encoded H.264 video streaming at low CPU cost, but can be used separately to ROSify your Pi camera modules. In order to achive maximum framerate on the Image topics, use YUV420 or Mono8 outputs. The additional BGR8 output costs extra CPU time as the node internally works with YUV420 and needs to scale up the U and V planes. Using the BGR8 output with H.264 is not recommended as it significantly degrades FPS.

The node can handle multiple cameras connected to the same board at the same time via different CSI ports (such as the Compute Module 4 or Pi 5).

> [!NOTE]
> Raspberry Pi 5 no longer has the hardware video encoder the older models had. Encoding to streamable H.264 is done at CPU cost.

## Install

### Install Docker, Docker Build & Docker Compose

E.g. on Debian/Ubuntu follow [these instructions](https://docs.docker.com/engine/install/debian/). Then add the current user to the docker group:
```bash
sudo usermod -aG docker ${USER}
# log out & back in
```

### (Optional) Clone this repo and build the Docker image from source

You can also use our pre-built Docker images, see [ghcr.io/phantomcybernetics/picam_ros2](https://ghcr.io/phantomcybernetics/picam_ros2) for ROS distributions (only ARM64 is provided as this package is meant to run on Raspberry Pi).

```bash
cd ~
git clone git@github.com:PhantomCybernetics/picam_ros2.git picam_ros2
cd picam_ros2
ROS_DISTRO=humble; docker build -f Dockerfile -t phntm/picam-ros2:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO .
```

### Configure
The following is an example config file (~/picam_ros2_params.yaml)
```yaml
/**:
  ros__parameters:
    # node_name: "picam_ros2" # name of the ROS node
    topic_prefix: '/picam_ros2/camera_'
    log_message_every_sec: 5.0
    log_scroll: False

    calibration_frames_needed: 10
    calibration_square_size_m: 0.0175 # set this to your actual calibration square dimension!
    calibration_pattern_size: [ 9, 6 ] # set this to your calibration chessboard size!
    calibration_files: '/calibration' # calibration files saved here

    /camera_0: # 0 is the camera location
      frame_id: 'pi_camera_optical_frame'

      enabled: True
      enable_calibration: True
      width: 1920
      height: 1080
      hflip: False
      vflip: False
      ae_enable: True # auto exposure enabled
      awb_enable: True # auto white-balance 
      contrast: 1.3
      analog_gain: 1.0 # analog gain of the sensor
      # exposure_time_ns: 30000 # manually set fixed exposure time if ae_enable=False
      # awb_mode: 0 # 0=auto, 1=incandescent, 2=tungsten, 3=fluorescent, 4=indoor, 5=daylight, 6=cloudy

      hw_encoder: True # True=using hw-encoder, False=CPU
      bitrate: 3000000
      compression: 30 # 0=no compression, 100=max
      framerate: 30
      # buffer_count: 4 # number of capture buffers

      publish_h264: True
      h264_reliability: RELIABLE # RELIABLE (default), BEST_EFFORT or SYSTEM_DEFAULT
      h264_durability: VOLATILE # TRANSIENT_LOCAL, VOLATILE (default) or SYSTEM_DEFAULT
      h264_history_depth: 10 # default 1

      publish_image: False 
      image_reliability: RELIABLE # RELIABLE (default), BEST_EFFORT or SYSTEM_DEFAULT
      image_durability: TRANSIENT_LOCAL # TRANSIENT_LOCAL, VOLATILE (default) or SYSTEM_DEFAULT
      image_history_depth: 1 # default 1

      publish_info: True
      info_reliability: BEST_EFFORT  # RELIABLE (default), BEST_EFFORT or SYSTEM_DEFAULT
      info_durability: VOLATILE # TRANSIENT_LOCAL, VOLATILE (default) or SYSTEM_DEFAULT
      info_history_depth: 1 # default 1
```

### Add service to your compose.yaml
```yaml
services:
  picam_ros2:
    image: ghcr.io/phantomcybernetics/picam_ros2:main-jazzy
    container_name: picam-ros2
    hostname: picam-ros2.local
    restart: unless-stopped
    privileged: true
    network_mode: host
    ipc: host
    # cpuset: '3' # consider restricting to a single CPU core
    # shm_size: 200m # more room for camera frames
    # environment:
    #  - ROS_DOMAIN_ID=22
    #  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # recommended with Phantom Bridge
    volumes:
      - ~/picam_ros2_params.yaml:/ros2_ws/picam_ros2_params.yaml # config goes here
      - ~/picam_ros2_calibration:/calibration # calibration files are stored here
      - /tmp:/tmp
      - /run/udev:/run/udev:ro # cameras need this
    devices:
      - /dev:/dev # cameras need this
    command:
      ros2 launch picam_ros2 picam_launch.py
```

### Launch
```bash
docker compose up picam_ros2
```

## Calibration

Camera needs to be calibrated before any CameraInfo messages can be published.
In order to calibrate a camera, you'll need a standard OpenCV calibratiion chessboard pattern [such as this one](https://raw.githubusercontent.com/opencv/opencv/refs/heads/4.x/doc/pattern.png) (more about these patterns can be found [here](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html)). Print or display it on a flat screen as large as possible, then make sure your `calibration_pattern_size` and `calibration_square_size_m` are set correctly in your YAML. You will need to restart the node/container to load the latest values from the YAML file. Attribute `publish_info` must be set to `True`.

Then call the `camera_N/calibrate` ROS service with `true` to start the calibration process. There's one such service for each detected camera, N represents the camera's location. [Phantom Bridge](https://docs.phntm.io/bridge) provides convenient UI to make these service calls, or even map them to keyboard keys or controller buttons.

Then start calling the `camera_N/sample_frame` service to capture individual calibration frames, the `calibration_frames_needed` attribute defines how many will be taken. Always make sure the calibration pattern is clearly visible, take samples from various angles and distances without any reflections, cover as much of the camera's field of view as possible.

After the last frame is captured, the node will process all of them (streaming will freeze for a while), then it'll start immediately publishing CameraInfo messages with the updated camera model. To save the calibration, call the `camera_N/save_calibration` service. The camera distortion coefficients will be saved in a JSON file in the directory configured by the `calibration_files` attribute and loaded on the next launch of the node. 

> [!TIP]
> If you map the `calibration_files` directory via `volumes` from the host filesystem to the Docker container as shown in the `compose.yaml` examples above, your calibration file will be preserved between Docker container rebuilds.

## Tested Hardware

| Board    | Encoder   | Camera                         | Resolution | Bitrate | FPS     |
| -------- | --------- | ------------------------------ | ---------- | ------- | ------- |
| Pi 4B    | BCM2711   | imx708_wide_noir               | 1920x1080  | 5000000 | 30      |
| CM 4     | BCM2711   | imx708_wide                    | 1920x1080  | 5000000 | 30      |
| CM 4     | BCM2711   | imx708_wide + imx708_wide_noir | 1280x720   | 5000000 | 30 + 30 |
| Pi 5     | CPU       | imx708_wide                    | 1920x1080  | 5000000 | 30      |
| Pi 5     | CPU       | imx708_wide + imx708_wide_noir | 1920x1080  | 5000000 | 30 + 30 |