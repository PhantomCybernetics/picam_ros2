#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include "picam_ros2.hpp"

#include "encoder_libav.hpp"
#include "encoder_hw.hpp"

#include "dma_heaps.hpp"
#include <linux/dma-buf.h>

#include "rclcpp/rclcpp.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavcodec/codec_id.h>
    #include <libavutil/opt.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}

class Encoder;

using namespace libcamera;

class CameraInterface {
    public:
        CameraInterface(std::shared_ptr<Camera> camera, std::shared_ptr<PicamROS2> node);
        void start();
        void stop();
        void publish(unsigned char *data, int size, bool keyframe, uint64_t pts, long timestamp_ns, bool log);
        ~CameraInterface();

        uint width;
        uint height;
        uint fps;
        uint bit_rate;
        uint compression;
        uint buffer_count;
        int bytes_per_pixel;
        int lines_printed = 0;

    private:
        std::shared_ptr<libcamera::Camera> camera;
        std::shared_ptr<PicamROS2> node;
        bool running = false;
        Encoder *encoder;

        std::vector<std::unique_ptr<Request>> capture_requests;
        
        size_t buffer_size; // whole buffer aligned to 4096
        int64_t frameIdx = 0;
        
        rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr publisher;
        
        // uint out_buffer_count;

        long log_message_every_ns;
        long last_log = 0;
        long timestamp_ns_base = 0;

        time_t last_fps_time = 0;
        int last_fps = 0;
        int frame_count = 0;

        bool log_scrolls = true;
        int location;
        int rotation;
        std::string model;

        std::string frame_id;
        
        bool hw_encoder;
 
        bool ae_enable;
        uint ae_exposure_mode;
        uint ae_metering_mode;
        uint ae_constraint_mode;
        std::vector<double> ae_constraint_mode_values; // [4]

        uint exposure_time;
        double analog_gain;
        bool awb_enable;
        std::vector<double> color_gains; // [2]
        double brightness;
        double contrast;

        uint stride;
        // AVFrame *frame;
        // AVPacket *packet;


        DmaHeap dma_heap;
        // std::map<Stream *, std::vector<std::unique_ptr<FrameBuffer>>> frame_buffers;
        // std::map<FrameBuffer *, std::vector<AVBufferRef *>> mapped_buffers;
        // std::map<FrameBuffer *, std::vector<uint>> mapped_buffer_strides;
        std::map<Stream *, std::vector<std::unique_ptr<FrameBuffer>>> capture_frame_buffers;
        std::map<FrameBuffer *, std::vector<AVBufferRef *>> mapped_capture_buffers;
        std::map<FrameBuffer *, std::vector<uint>> mapped_capture_buffer_strides;

        ffmpeg_image_transport_msgs::msg::FFMPEGPacket outFrameMsg;

        StreamConfiguration *streamConfig;
        void readConfig();
        // void eventLoop();
        // bool initializeSWEncoder();
        // bool initializeHWEncoder();
        // void frameRequestComplete(Request *request);
        void captureRequestComplete(Request *request);
        // int resetEncoder(const char* device_path);
};