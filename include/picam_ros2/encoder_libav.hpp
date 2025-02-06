#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include "encoder_base.hpp"
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

using namespace libcamera;

class EncoderLibAV : public Encoder {
    public:
        EncoderLibAV(CameraInterface *interface, std::shared_ptr<libcamera::Camera> camera);
        ~EncoderLibAV();
        void encode(std::vector<AVBufferRef *> plane_buffers, std::vector<uint> plane_strides, int base_fd, uint size, int64_t *frameIdx, long timestamp_ns, bool log);

    private:
        AVCodec *codec;
        AVCodecContext *codec_context;
        AVFrame *frame_to_encode;
        AVPacket *encoded_packet;
};