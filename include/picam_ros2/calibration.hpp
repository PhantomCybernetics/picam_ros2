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
void calibrateCamera(const std::vector<cv::Mat>& images, cv::Size pattern_size, float square_size, sensor_msgs::msg::CameraInfo& camera_info);
cv::Mat yuv420ToMonoCopy(const std::vector<AVBufferRef *>& planes, const std::vector<unsigned int>& strides, uint width, uint height);

bool readCalibration(std::string file_name, sensor_msgs::msg::CameraInfo& camera_info, std::string camera_model, int width, int height);
bool writeCalibration(sensor_msgs::msg::CameraInfo& camera_info, std::string camera_model, int width, int height, std::string file_name);