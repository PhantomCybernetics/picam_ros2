#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include "picam_ros2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavcodec/codec_id.h>
    #include <libavutil/opt.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}

const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string BLUE = "\033[34m";
const std::string YELLOW = "\033[33m";
const std::string MAGENTA = "\033[35m";
const std::string CYAN = "\033[36m";
const std::string WHITE = "\033[37m2";
const std::string CLR = "\033[0m";

using namespace libcamera;

class CameraInterface {
    public:
        // int width = 1280;
        // int height = 720;
        int width = 1920;
        int height = 1080;
        int fps = 30;

        CameraInterface(std::shared_ptr<Camera> camera, std::shared_ptr<PicamROS2> node);
        ~CameraInterface();

    private:
        std::shared_ptr<libcamera::Camera> camera;
        std::shared_ptr<PicamROS2> node;
        std::vector<std::unique_ptr<Request>> requests;

        AVCodec *codec;
        AVCodecContext *codec_context;
        int frameIdx = 0;
        int bytes_per_pixel;
        rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr publisher;
        
        StreamConfiguration streamConfig;
        bool initializeEncoder();
        void requestComplete(Request *request);
};