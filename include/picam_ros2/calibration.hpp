#pragma once

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavcodec/codec_id.h>
    #include <libavutil/opt.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/camera_info.hpp"

cv::Mat yuv420ToRgbCopy(const std::vector<AVBufferRef *>& planes, const std::vector<unsigned int>& strides, uint width, uint height);
sensor_msgs::msg::CameraInfo calibrateCamera(const std::vector<cv::Mat>& images, cv::Size patternSize, float squareSize);
