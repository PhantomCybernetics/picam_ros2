// #include <chrono>
// #include <future>
// #include <memory>
// #include <string>
// #include <bitset>

// #include <libcamera/libcamera.h>
// #include <libcamera/pixel_format.h>

// #include <sys/mman.h>
// #include <unistd.h>
// #include <fmt/core.h>
// #include <libcamera/base/shared_fd.h>

// #include "libcamera/camera_manager.h"
// #include "libcamera/control_ids.h"
// #include "libcamera/property_ids.h"

#include <bitset>

#include "picam_ros2/encoder_libav.hpp"
#include "picam_ros2/camera_interface.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/qos.hpp"
// #include "std_msgs/msg/header.hpp"
// #include "builtin_interfaces/msg/time.hpp"
// #include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"

// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/videodev2.h>


extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavcodec/codec_id.h>
    #include <libavutil/opt.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}

using namespace libcamera;

EncoderLibAV::EncoderLibAV(CameraInterface *interface, std::shared_ptr<libcamera::Camera> camera)
    : Encoder(interface, camera) {

    std::cout << BLUE << "Using SW encoder" << CLR << std::endl;
    this->codec = avcodec_find_encoder(AV_CODEC_ID_H264); //CPU
    
    if (!this->codec){
        std::cerr << "Codec with specified id not found" << std::endl;
        return;
    }
    this->codec_context = avcodec_alloc_context3(codec);
    if (!this->codec_context){
        std::cerr << "Can't allocate video codec context" << std::endl;
        return;
    }

    assert(this->interface->width % 32 == 0 && "Width not aligned to 32");

    // this->codec_context->profile = FF_PROFILE_H264_CONSTRAINED_BASELINE;
    this->codec_context->profile = FF_PROFILE_H264_HIGH;
    this->codec_context->height = this->interface->height;
    this->codec_context->width = this->interface->width;

    /// Frames per second
    this->codec_context->time_base.num = 1;
    this->codec_context->time_base.den = this->interface->fps;
    this->codec_context->framerate.num = this->interface->fps;
    this->codec_context->framerate.den = 1;

    this->codec_context->bit_rate = this->interface->bit_rate;

    /// Only YUV420P for H264|5
    this->codec_context->pix_fmt = AV_PIX_FMT_YUV420P;

    /// Key(intra) frame rate
    this->codec_context->gop_size = this->interface->fps*2;

    /// P-frames, generated by referencing data from prev and future frames.
    /// [Compression up, CPU usage up]
    /// [use 3/gop]
    this->codec_context->max_b_frames = 0;

    /// Can be used by a P-frame(predictive, partial frame) to help define a future frame in a compressed video.
    /// [use 3–5 ref per P]
    this->codec_context->refs = 0;

    /// Compression efficiency (slower -> better quality + higher cpu%)
    /// [ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow]
    /// Set this option to "ultrafast" is critical for realtime encoding
    av_opt_set(this->codec_context->priv_data, "preset", "ultrafast", 0);
    
    // std::string n_buffs = fmt::format("{}", );
    av_opt_set_int(codec_context->priv_data, "num_capture_buffers", this->interface->buffer_count, 0);
    av_opt_set(codec_context->priv_data, "input-io-mode", "dmabuf", 0);

    /// Compression rate (lower -> higher compression) compress to lower size, makes decoded image more noisy
    /// Range: [0; 51], sane range: [18; 26]. I used 35 as good compression/quality compromise. This option also critical for realtime encoding
    // std::string crf = fmt::format("{}", this->compression);
    // av_opt_set(this->codec_context->priv_data, "crf", crf.c_str(), 0);
    av_opt_set_int(this->codec_context->priv_data, "crf", this->interface->compression, 0);

    av_opt_set_int(codec_context->priv_data, "forced-idr", 1, 0);

    /// Change settings based upon the specifics of input
    /// [psnr, ssim, grain, zerolatency, fastdecode, animation]
    /// This option is most critical for realtime encoding, because it removes delay between 1th input frame and 1th output packet.
    av_opt_set(this->codec_context->priv_data, "tune", "zerolatency", 0);

    auto desc = av_pix_fmt_desc_get(AV_PIX_FMT_DRM_PRIME);
    if (!desc){
        std::cerr << "Can't get descriptor for pixel format for AV_PIX_FMT_YUV420P" << std::endl;
        return;
    }
    this->interface->bytes_per_pixel = av_get_bits_per_pixel(desc) / 8;
    
    std::cerr << CYAN << "Encoder initiated for " << this->interface->width << "x" << this->interface->height << " @ " << this->interface->fps << " fps" << "; BPP=" << this->interface->bytes_per_pixel << CLR << std::endl;

    if(avcodec_open2(this->codec_context, this->codec, nullptr) < 0){
        std::cerr << "Could not open codec" << std::endl;
        return;
    }

    this->frame_to_encode = av_frame_alloc();
    if (!this->frame_to_encode){
        std::cerr << "Could not allocate video frame" << std::endl;
        return;
    }
    this->frame_to_encode->format = this->codec_context->pix_fmt;
    this->frame_to_encode->height = this->codec_context->height;
    this->frame_to_encode->width = this->codec_context->width;

    this->encoded_packet = av_packet_alloc();
    if (this->encoded_packet == NULL) {
        std::cerr << RED << "Error making packet" << CLR << std::endl;
        return;
    }
}

void EncoderLibAV::encode(std::vector<AVBufferRef *> plane_buffers, std::vector<uint> plane_strides, int, uint, int64_t *frameIdx, long timestamp_ns, bool log) {

    for (size_t i = 0; i < 3; ++i) {
        this->frame_to_encode->buf[i] = plane_buffers[i];
        this->frame_to_encode->data[i] = this->frame_to_encode->buf[i]->data;
        this->frame_to_encode->linesize[i] = plane_strides[i];
    }

    /// Set frame index in range: [1, fps]
    this->frame_to_encode->pts = *frameIdx;

    /// Set frame type
    bool isKeyFrame = false; // this->frameIdx == 0;
    if (isKeyFrame){
        this->frame_to_encode->key_frame = 1;
        this->frame_to_encode->pict_type = AVPictureType::AV_PICTURE_TYPE_I;
    }

    bool frame_ok = false;
    switch (avcodec_send_frame(this->codec_context, this->frame_to_encode)){
        case 0:
            frame_ok = true;
            *frameIdx = ((*frameIdx) % this->codec_context->framerate.num) + 1;
            break;
        case AVERROR(EAGAIN):
            this->interface->err("Error sending frame to encoder: AVERROR(EAGAIN)");
            break;
        case AVERROR_EOF:
            this->interface->err("Error sending frame to encoder: AVERROR_EOF");
            break;
        case AVERROR(EINVAL):
            this->interface->err("Error sending frame to encoder: AVERROR(EINVAL)");
            break;
        case AVERROR(ENOMEM):
            this->interface->err("Error sending frame to encoder: AVERROR(ENOMEM)");
            break;
        default:
            this->interface->err("Error sending frame to encoder: Other error");
            break;
    }

    // if (munmap(buf_base, total_size) == -1) {
    //     std::cerr << "munmap failed: " << strerror(errno) << std::endl;
    // }
    // buf_base = nullptr;  // Optional: prevent accidental reuse

    if (!frame_ok) {
        return;
    }

    bool packet_ok = false;
    switch (avcodec_receive_packet(this->codec_context, this->encoded_packet)) {
        case 0:
            /// use packet, copy/send it's data, or whatever
            packet_ok = true;
            if (log) {
                auto clr = this->encoded_packet->flags == 1 ? MAGENTA : YELLOW;
                this->interface->log(clr, "PACKET ", this->encoded_packet->size,
                                     " / ", this->encoded_packet->buf->size,
                                     " pts=", this->encoded_packet->pts,
                                     " flags=", this->encoded_packet->flags);
            }
            break;
        case AVERROR(EAGAIN):
            this->interface->err("Error receiving packet AVERROR(EAGAIN)");
            break;
        case AVERROR_EOF:
            this->interface->err("Error receiving packet AVERROR_EOF");
            break;
        case AVERROR(EINVAL):
            this->interface->err("Error receiving packet AVERROR(EINVAL)");
            break;
        default:
            this->interface->err("Error receiving packet");
            break;
    }

    if (this->encoded_packet->flags & AV_PKT_FLAG_CORRUPT) {
        this->interface->err("Packed flagged as corrupt");
        return;
    }

    if (this->encoded_packet->flags != 0 && this->encoded_packet->flags != AV_PKT_FLAG_KEY) {
        this->interface->err(MAGENTA, "Packed flags:", std::bitset<4>(this->encoded_packet->flags));
    }
    
    if (packet_ok) {
        uint64_t pts = av_rescale_q(this->encoded_packet->pts, //this->outFrameMsg.header.stamp.sec * NS_TO_SEC + this->outFrameMsg.header.stamp.nanosec, //this->packet->pts
                                    codec_context->time_base,
                                    AVRational{1, CLOCK_RATE});
        // uint64_t pts = this->encoded_packet->pts; //this->encoded_packet->pts * CLOCK_RATE) / NS_TO_SEC;
        bool keyframe = !!(this->encoded_packet->flags & AV_PKT_FLAG_KEY);
        this->interface->publishH264(this->encoded_packet->data, this->encoded_packet->size, keyframe, pts, timestamp_ns, log);
    }
    
}

EncoderLibAV::~EncoderLibAV() {
    std::cout << BLUE << "Cleaning up sw encoder" << CLR << std::endl;

    av_frame_unref(this->frame_to_encode);
    av_frame_free(&this->frame_to_encode);
    av_packet_unref(this->encoded_packet);
    av_packet_free(&this->encoded_packet);
    
    avcodec_close(this->codec_context);
    avcodec_free_context(&this->codec_context);
}