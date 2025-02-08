#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <fmt/core.h>

#include <opencv2/opencv.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>
#include <libcamera/transform.h>

#include "picam_ros2.hpp"

#include "encoder_libav.hpp"
#include "encoder_hw.hpp"

#include "dma_heaps.hpp"
#include <linux/dma-buf.h>

#include "rclcpp/rclcpp.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class Encoder;

using namespace libcamera;

class CameraInterface {
    public:
        CameraInterface(std::shared_ptr<Camera> camera, int location, int rotation, std::string model, std::shared_ptr<PicamROS2> node);
        void start();
        void stop();
        void publishH264(unsigned char *data, int size, bool keyframe, uint64_t pts, long timestamp_ns, bool log);
        void publishImage(const std::vector<AVBufferRef *>& planes, const std::vector<unsigned int>& strides, uint buffer_size, long timestamp_ns, bool log);
        void publishCameraInfo(long timestamp_ns, bool log);

        ~CameraInterface();

        uint width;
        uint height;
        uint fps;
        uint bit_rate;
        uint compression;
        uint buffer_count;
        int bytes_per_pixel;

        template<typename... Args>
        void log(const Args&... args) {
            std::ostringstream oss;
            (oss << ... << args);
            std::cout << oss.str() << CLR << std::endl;
            this->lines_printed++;
        }
        template<typename... Args>
        void err(const Args&... args) {
            std::ostringstream oss;
            (oss << ... << args);
            std::cerr << RED << oss.str() << CLR << std::endl;
            this->lines_printed = -1; // don't erase on err
        }
        static std::string GetConfigPrefix(int location) {
            return fmt::format("/camera_{}.", location);
        }

    private:
        int lines_printed = 0;
        
        std::shared_ptr<libcamera::Camera> camera;

        std::shared_ptr<PicamROS2> node;

        bool publish_h264;
        bool publish_image;
        uint image_output_format;
        bool publish_info;

        std::string h264_topic;
        std::string info_topic;
        std::string image_topic;

        bool running = false;
        Encoder *encoder;

        std::vector<std::unique_ptr<Request>> capture_requests;
        
        size_t buffer_size; // whole buffer aligned to 4096
        int64_t frame_idx = 0;
        
        rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr h264_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher;
        
        ffmpeg_image_transport_msgs::msg::FFMPEGPacket out_h264_msg;
        sensor_msgs::msg::Image out_image_msg;
        sensor_msgs::msg::CameraInfo out_info_msg;

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
        
        bool hw_encoder, hflip, vflip;

        bool ae_enable;
        uint ae_exposure_mode;
        uint ae_metering_mode;
        uint ae_constraint_mode;
        std::vector<double> ae_constraint_mode_values; // [4]

        uint exposure_time;
        double analog_gain;
        bool awb_enable;
        // bool awb_locked;
        uint awb_mode;
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

        bool enable_calibration, calibration_running = false;
        void calibration_toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        void calibration_sample_frame(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void calibration_save(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> srv_calibration_toggle;
        std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv_calibration_sample_frame;
        std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv_calibration_save;
        uint calibration_frames_requested;
        uint calibration_frames_needed;
        long last_calibration_frame_taken_ns = 0;
        const uint calibration_min_frame_delay_ns = 1000;
        std::vector<cv::Mat> calibration_frames;
        cv::Size calibration_pattern_size;
        float calibration_square_size;
        std::string calibration_files_base_path;
        std::string calibration_file;

        StreamConfiguration *streamConfig;
        void readConfig();
        // void eventLoop();
        // bool initializeSWEncoder();
        // bool initializeHWEncoder();
        // void frameRequestComplete(Request *request);
        void captureRequestComplete(Request *request);
        // int resetEncoder(const char* device_path);
};