#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include "picam_ros2.hpp"
#include "dma_heaps.hpp"

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

const int NS_TO_SEC = 1000000000;

using namespace libcamera;

class CameraInterface {
    public:
        CameraInterface(std::shared_ptr<Camera> camera, std::shared_ptr<PicamROS2> node);
        void start();
        void stop();
        ~CameraInterface();

    private:
        std::shared_ptr<libcamera::Camera> camera;
        std::shared_ptr<PicamROS2> node;
        std::vector<std::unique_ptr<Request>> requests;

        bool running = false;
        AVCodec *codec;
        AVCodecContext *codec_context;
        int frameIdx = 0;
        int bytes_per_pixel;
        rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr publisher;
        int lines_printed = 0;
        uint buffer_count;
        double log_message_every_sec;
        time_t last_log = 0;
        
        time_t last_fps_time = 0;
        int last_fps = 0;
        int frame_count = 0;

        bool log_scrolls = true;
        int location;
        int rotation;
        std::string model;

        uint width;
        uint height;
        bool hw_encoder;
        int fps = 30;
        int bit_rate;
        int compression;
        std::string frame_id;

        uint stride;
        AVFrame *frame;
        AVPacket *packet;

        DmaHeap dma_heap;
        std::map<Stream *, std::vector<std::unique_ptr<FrameBuffer>>> frame_buffers;
        std::map<FrameBuffer *, std::vector<AVBufferRef *>> mapped_buffers;
        std::map<FrameBuffer *, std::vector<uint>> mapped_buffer_strides;

        ffmpeg_image_transport_msgs::msg::FFMPEGPacket outFrameMsg;
        
        StreamConfiguration *streamConfig;
        void readConfig();
        void eventLoop();
        bool initializeEncoder();
        void frameRequestComplete(Request *request);
        int resetEncoder(const char* device_path);
};