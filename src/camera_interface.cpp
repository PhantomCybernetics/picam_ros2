#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include <sys/mman.h>
#include <unistd.h>
#include <fmt/core.h>

#include "libcamera/property_ids.h"
#include "picam_ros2/picam_ros2.hpp"
#include "picam_ros2/camera_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavcodec/codec_id.h>
    #include <libavutil/opt.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}

using namespace libcamera;

CameraInterface::CameraInterface(std::shared_ptr<Camera> camera, std::shared_ptr<PicamROS2> node) {

    // this->resetEncoder("/dev/video11");

    std::cout << GREEN << "Initiating " << camera->id() << CLR << std::endl; 

    this->camera = camera;
    this->node = node;

    // inspect camera
    this->camera->acquire();
    const ControlList &props = camera->properties();
    if (props.contains(properties::Location.id())) {
        this->location = props.get(properties::Location).value();
    }
    if (props.contains(properties::Model.id())) {
        this->model = props.get(properties::Model)->c_str();
    }
    if (props.contains(properties::Rotation.id())) {
        this->rotation = props.get(properties::Rotation).value();
    }  

    // declare & read configs
    std::string config_prefix = fmt::format("/camera_{}.", this->location);
    
    this->node->declare_parameter(config_prefix + "hflip", false);
    this->node->declare_parameter(config_prefix + "vflip", false);

    this->node->declare_parameter(config_prefix + "hw_encoder", true);
    this->hw_encoder = this->node->get_parameter(config_prefix + "hw_encoder").as_bool();

    this->node->declare_parameter(config_prefix + "bitrate", 4000000);
    this->bit_rate = this->node->get_parameter(config_prefix + "bitrate").as_int();

    this->node->declare_parameter(config_prefix + "framerate", 30);
    this->fps = this->node->get_parameter(config_prefix + "framerate").as_int();

    this->node->declare_parameter(config_prefix + "frame_id", "picam");
    this->frame_id = this->node->get_parameter(config_prefix + "frame_id").as_string();

    this->log_scrolls = this->node->get_parameter("log_scroll").as_bool();
    
    // configure camera
    std::unique_ptr<CameraConfiguration> config = this->camera->generateConfiguration( { StreamRole::VideoRecording } );
    config->validate();
    
    this->streamConfig = config->at(0);
    std::cout << YELLOW << "Camera model: " << this->model << " Location: " << this->location << " Rotation: " << this->rotation << CLR << std::endl; 
    std::cout << YELLOW << "Camera orinetation: " << config->orientation << CLR << std::endl; 
    std::cout << YELLOW << "Stream config: " << this->streamConfig.toString() << CLR << std::endl; 
    std::cout << YELLOW << "Stride: " << this->streamConfig.stride << CLR << std::endl; 
    std::cout << YELLOW << "Bit rate: " << this->bit_rate << CLR << std::endl; 
    this->camera->configure(config.get());

    FrameBufferAllocator *allocator = new FrameBufferAllocator(this->camera);

    std::cout << "Allocating..." << std::endl;
    for (StreamConfiguration &cfg : *config) {
        auto stream = cfg.stream();
        int ret = allocator->allocate(stream);
        if (ret < 0) {
            std::cerr << "Can't allocate buffers" << std::endl;
            return;
        }
        const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
        size_t allocated = buffers.size();
        std::cout << "Allocated " << allocated << " buffers for stream pixel format: " << cfg.pixelFormat.toString() << std::endl;

        for (unsigned int i = 0; i < buffers.size(); ++i) {
            std::unique_ptr<Request> request = camera->createRequest();
            if (!request)
            {
                std::cerr << "Can't create request" << std::endl;
                return;
            }

            const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
            int ret = request->addBuffer(stream, buffer.get());
            if (ret < 0)
            {
                std::cerr << "Can't set buffer for request" << std::endl;
                return;
            }
            this->requests.push_back(std::move(request));
        }
    }

    this->camera->requestCompleted.connect(this, &CameraInterface::requestComplete);
    this->camera->start();

    // init frame producer
    std::string topic = fmt::format(this->node->get_parameter("topic_prefix").as_string() + "{}/{}", this->location, this->model);
    std::cout << "Creatinng publisher for " << topic << std::endl;
    auto qos = rclcpp::QoS(1);
    this->publisher = this->node->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(topic, qos);

    // init encoder
    if (!this->initializeEncoder()) {
        std::cerr << "Error initializing encoder" << std::endl; 
        return;
    }
    
    if(avcodec_open2(this->codec_context, this->codec, nullptr) < 0){
        std::cerr << "Could not open codec" << std::endl;
        return;
    }

    if (this->hw_encoder) { // wait a bit to allow encoder init
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    for (std::unique_ptr<Request> &request : this->requests)
        this->camera->queueRequest(request.get());
}

bool CameraInterface::initializeEncoder() {
    // Initialize encoder
    if (this->hw_encoder) {
        std::cout << GREEN << "Using HW encoder" << CLR << std::endl;
        this->codec = avcodec_find_encoder_by_name("h264_v4l2m2m"); // hw encoder on bcm2835
    } else {
        std::cout << GREEN << "Using CPU encoder" << CLR << std::endl;
        this->codec = avcodec_find_encoder(AV_CODEC_ID_H264); //CPU
    }
    if (!this->codec){
        std::cerr << "Codec with specified id not found" << std::endl;
        return false;
    }
    this->codec_context = avcodec_alloc_context3(codec);
    if (!this->codec_context){
        std::cerr << "Can't allocate video codec context" << std::endl;
        return false;
    }

    assert(this->width % 32 == 0 && "Width not aligned to 32");

    this->codec_context->profile = FF_PROFILE_H264_CONSTRAINED_BASELINE;
    this->codec_context->height = this->height;
    this->codec_context->width = this->width;

    /// Frames per second
    this->codec_context->time_base.num = 1;
    this->codec_context->time_base.den = this->fps;
    this->codec_context->framerate.num = this->fps;
    this->codec_context->framerate.den = 1;

    this->codec_context->bit_rate = this->bit_rate;

    /// Only YUV420P for H264|5
    this->codec_context->pix_fmt = AV_PIX_FMT_YUV420P;

    /// Key(intra) frame rate
    this->codec_context->gop_size = this->fps*2;

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

    /// Compression rate (lower -> higher compression) compress to lower size, makes decoded image more noisy
    /// Range: [0; 51], sane range: [18; 26]. I used 35 as good compression/quality compromise. This option also critical for realtime encoding
    av_opt_set(this->codec_context->priv_data, "crf", "35", 0);

    av_opt_set(codec_context->priv_data, "forced-idr", "1", 0);

    /// Change settings based upon the specifics of input
    /// [psnr, ssim, grain, zerolatency, fastdecode, animation]
    /// This option is most critical for realtime encoding, because it removes delay between 1th input frame and 1th output packet.
    av_opt_set(this->codec_context->priv_data, "tune", "zerolatency", 0);

    auto desc = av_pix_fmt_desc_get(AV_PIX_FMT_YUV420P);
    if (!desc){
        std::cerr << "Can't get descriptor for pixel format for AV_PIX_FMT_YUV420P" << std::endl;
        return false;
    }
    this->bytes_per_pixel = av_get_bits_per_pixel(desc) / 8;
    
    std::cerr << CYAN << "Encoder initiated for " << this->width << "x" << this->height << " @ " << this->fps << " fps" << "; BPP=" << this->bytes_per_pixel << CLR << std::endl;

    return true;
}

void get_current_stamp(builtin_interfaces::msg::Time *stamp, uint64_t timestamp_ns) {
    // Split into seconds and nanoseconds
    stamp->sec = static_cast<int32_t>(timestamp_ns / NS_TO_SEC);
    stamp->nanosec = static_cast<uint32_t>(timestamp_ns % NS_TO_SEC);;
}

void CameraInterface::requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled)
        return;
    
    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        // unsigned int nplane = 0;
        // std::cout << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence << " bytesused: ";
        // for (const FrameMetadata::Plane &plane : metadata.planes())  {
            // std::cout << plane.bytesused;
            // if (++nplane < metadata.planes().size()) std::cout << "/";
        // }
        // std::cout << std::endl;

        AVFrame *frame = av_frame_alloc();
        if (!frame){
            std::cerr << "Could not allocate video frame" << std::endl;
            return;
        }

        frame->format = this->codec_context->pix_fmt;
        frame->height = this->codec_context->height;
        frame->width = this->codec_context->width;

        const std::vector<libcamera::FrameBuffer::Plane>& planes = buffer->planes();

        size_t total_size = 0;
        for (size_t i = 0; i < planes.size(); ++i) {
            size_t plane_end = planes[i].offset + planes[i].length;
            if (plane_end > total_size) {
                total_size = plane_end;
            }
        }

        void* base = mmap(nullptr, total_size, PROT_READ, MAP_SHARED, planes[0].fd.get(), 0);
        if (base == MAP_FAILED) {
            std::cerr << "Failed to mmap entire buffer: " << strerror(errno) << std::endl;
            return;
        }

        auto free_buffer = [](void* opaque, uint8_t* data) {
            munmap(data, static_cast<size_t>(reinterpret_cast<uintptr_t>(opaque)));
        };

        if (this->verbose) {
            if (!this->log_scrolls && this->lines_printed > 0) {
                for (int i = 0; i < this->lines_printed; i++) {
                    std::cout << std::string(this->lines_printed, '\033') << "[A\033[K";
                }
            }
            this->lines_printed = 0;
        }

        if (this->verbose)
            std::cout << std::setw(6) << std::setfill('0') << metadata.sequence << ": ";

        for (size_t i = 0; i < planes.size(); ++i) {

            // Map the memory
            if (this->verbose) {
                std::cout << "plane#" << i <<" ";
                if (i == 0)
                    std::cout << "fd=" << planes[i].fd.get() << "; ";
                std::cout << planes[i].offset << "+" << planes[i].length << " ";
            }
            
            frame->buf[i] = av_buffer_create(
                static_cast<uint8_t*>(base)+planes[i].offset,
                planes[i].length,
                free_buffer,
                reinterpret_cast<void*>(planes[i].length),
                0
            );

            // Assign plane data using offsets within the mapped buffer
            frame->data[i] = frame->buf[i]->data;
            frame->linesize[i] = (i == 0) ? this->streamConfig.stride : this->streamConfig.stride / 2;

            if (this->verbose) {
                std::cout << "|" << frame->linesize[i] << "|";
                if (i < planes.size()-1)
                    std::cout << "; ";
            }
        }
        if (this->verbose) {
            std::cout << std::endl;
            this->lines_printed++;
        }

        /// Set frame index in range: [1, fps]
        frame->pts = this->frameIdx;

        /// Set frame type
        bool isKeyFrame = false;
        if (isKeyFrame){
            frame->key_frame = 1;
            frame->pict_type = AVPictureType::AV_PICTURE_TYPE_I;
        }
        // std::cout << "Sending..." << std::endl;

        bool frame_ok = false;
        switch (avcodec_send_frame(this->codec_context, frame)){
            case 0:
                frame_ok = true;
                this->frameIdx = (this->frameIdx % this->codec_context->framerate.num) + 1;
                break;
            case AVERROR(EAGAIN):
                std::cerr << RED << "Error sending frame to encoder: AVERROR(EAGAIN)" << CLR << std::endl;
                this->lines_printed = -1;
                break;
            case AVERROR_EOF:
                std::cerr << RED << "Error sending frame to encoder: AVERROR_EOF" << CLR << std::endl;
                this->lines_printed = -1;
                break;
            case AVERROR(EINVAL):
                std::cerr << RED << "Error sending frame to encoder: AVERROR(EINVAL)" << CLR << std::endl;
                this->lines_printed = -1;
                break;
            case AVERROR(ENOMEM):
                std::cerr << RED << "Error sending frame to encoder: AVERROR(ENOMEM)" << CLR << std::endl;
                this->lines_printed = -1;
                break;
            default:
                std::cerr << RED << "Error sending frame to encoder: Other error" << CLR << std::endl;
                this->lines_printed = -1;
                break;
        }

        av_frame_unref(frame);
        av_frame_free(&frame);

        if (!frame_ok) {
            continue;
        }

        AVPacket *packet = av_packet_alloc();
        if (packet == NULL) {
            std::cerr << RED << "Error making packet" << CLR << std::endl;
            this->lines_printed = -1;
            continue;
        }

        bool packet_ok = false;
        switch (avcodec_receive_packet(this->codec_context, packet)) {
            case 0:
                /// use packet, copy/send it's data, or whatever
                packet_ok = true;
                if (this->verbose) {
                    std::cout << (packet->flags == 1 ? MAGENTA : YELLOW);
                    
                    std::cout << "PACKET " << packet->size
                            << " / " << packet->buf->size
                            << " pts=" << packet->pts
                            << " flags=" << packet->flags;
                    std::cout << CLR;
                    std::cout << std::endl;
                    this->lines_printed++;
                }
                break;
            case AVERROR(EAGAIN):
                std::cerr << RED << "Error receiving packet AVERROR(EAGAIN)" << CLR << std::endl;
                this->lines_printed = -1;
                break;
            case AVERROR_EOF:
                std::cerr << RED << "Error receiving packet AVERROR_EOF" << CLR << std::endl;
                this->lines_printed = -1;
                break;
            case AVERROR(EINVAL):
                std::cerr << RED << "Error receiving packet AVERROR(EINVAL)" << CLR << std::endl;
                this->lines_printed = -1;
                break;
            default:
                std::cerr << RED << "Error receiving packet" << CLR << std::endl;
                this->lines_printed = -1;
                break;
        }

        if (packet_ok) {
            ffmpeg_image_transport_msgs::msg::FFMPEGPacket outFrameMsg;
            std_msgs::msg::Header header;
            header.frame_id = this->frame_id;
            header.stamp = builtin_interfaces::msg::Time();
            uint64_t timestamp_ns = metadata.timestamp;
            get_current_stamp(&header.stamp, timestamp_ns);
            outFrameMsg.header = header;
            outFrameMsg.width = this->codec_context->width;
            outFrameMsg.height = this->codec_context->height;
            outFrameMsg.encoding = "h.264";
            outFrameMsg.pts = av_rescale_q(packet->pts /*header.stamp.sec * NS_TO_SEC + header.stamp.nanosec*/,
                                           codec_context->time_base,
                                           AVRational{1, 90000});

            outFrameMsg.flags = packet->flags;
            outFrameMsg.is_bigendian = false;
            // outFrameMsg.data = &packet->data; // uint8[] out
            // outFrameMsg.data = std::vector<uint8_t>(packet->size);
            outFrameMsg.data.assign(packet->data, packet->data + packet->size);

            if (this->verbose) {
                std::cout << GREEN << " >> Sending " << outFrameMsg.data.size() << "B" << CLR << " sec: " << outFrameMsg.header.stamp.sec << " nsec:" << outFrameMsg.header.stamp.nanosec << std::endl;
                this->lines_printed++;
            }
        
            this->publisher->publish(outFrameMsg);
        }

        av_packet_unref(packet);
        av_packet_free(&packet);   
    }

    request->reuse(Request::ReuseBuffers);
    this->camera->queueRequest(request);
}

int CameraInterface::resetEncoder(const char* device_path) {
    int fd = open(device_path, O_RDWR);
    if (fd < 0) {
        perror("Failed to open V4L2 device");
        return -1;
    }

    // Stop streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("Failed to stop streaming");
    }

    // Reset the device (optional, driver-dependent)
    // Some drivers support VIDIOC_RESET, but it's not universal
    if (ioctl(fd, _IOW('V', 99, int), 0) < 0) {
        // Fallback: Close and reopen the device
        close(fd);
        fd = open(device_path, O_RDWR);
    }

    close(fd);
    return 0;
}

CameraInterface::~CameraInterface() {
    this->camera->stop();
    this->camera->release();
    this->camera = NULL;
    this->node = NULL;
    avcodec_close(this->codec_context);
    avcodec_free_context(&this->codec_context);
}