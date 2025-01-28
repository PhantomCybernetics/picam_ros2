#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include <sys/mman.h>
#include <unistd.h>
#include <fmt/core.h>
#include <linux/dma-buf.h>
#include <libcamera/base/shared_fd.h>

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


void free_buffer(void* opaque, uint8_t* data) {
    munmap(data, static_cast<size_t>(reinterpret_cast<uintptr_t>(opaque)));
}

CameraInterface::CameraInterface(std::shared_ptr<Camera> camera, std::shared_ptr<PicamROS2> node) {

    // this->resetEncoder("/dev/video11");

    this->camera = camera;
    this->node = node;
    
}

// void ReplaceDMABufs(libcamera::FrameBuffer framebuffer)
// {
// 	auto planes = framebuffer.planes();
// 	for(auto plane : planes)
// 	{
// 		const auto& DMABufferName = std::string("dma") + std::to_string(BufferIndex++); // <-- Not really important
// 		const auto AlignedPlaneSize = J3Maths::RoundUp(Plane.length, 4096); // TODO: obtain alignment size programmatically
// 		auto DMAfd = libcamera::SharedFD(DMAHeap->Allocate(DMABufferName, AlignedPlaneSize));
// 		if (!DMAfd.isValid())
// 			error...
// 		Plane.fd = DMAfd;
// 	}
// }

uint32_t roundUp4096(uint32_t x) {
    constexpr uint32_t mask = 4096 - 1; // 0xFFF
    return (x + mask) & ~mask;
}

void CameraInterface::start() {
    if (this->running)
        return;
    this->running = true;

    std::cout << GREEN << "Initializing " << camera->id() << CLR << std::endl; 

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

    // declare params & read configs
    this->readConfig();
    
    // configure camera
    std::unique_ptr<CameraConfiguration> config = this->camera->generateConfiguration( { StreamRole::VideoRecording } );
    
    this->streamConfig = &config->at(0);
    this->streamConfig->size.width = this->width;
    this->streamConfig->size.height = this->height;
    this->streamConfig->bufferCount = this->buffer_count;
    // this->streamConfig.stride = (uint)this->width;

    if (config->validate() == CameraConfiguration::Invalid)
	    throw std::runtime_error("Failed to validate stream configurations");

    this->stride = this->streamConfig->stride;

    std::cout << YELLOW << "Camera model: " << this->model << " Location: " << this->location << " Rotation: " << this->rotation << CLR << std::endl; 
    std::cout << YELLOW << "Camera orinetation: " << config->orientation << CLR << std::endl; 
    std::cout << YELLOW << "Stream config: " << this->streamConfig->toString() << CLR << std::endl; 
    std::cout << YELLOW << "Stride: " << this->streamConfig->stride << CLR << std::endl; 
    std::cout << YELLOW << "Bit rate: " << this->bit_rate << CLR << std::endl; 
    std::cout << YELLOW << "Compression rate: " << this->compression << CLR << std::endl; 
    std::cout << YELLOW << "Buffer count: " << this->streamConfig->bufferCount << CLR << std::endl; 
    this->camera->configure(config.get());

    // FrameBufferAllocator *allocator = new FrameBufferAllocator(this->camera);

    std::cout << "Allocating..." << std::endl;
    for (StreamConfiguration &cfg : *config) {
        auto stream = cfg.stream();

        std::vector<std::unique_ptr<FrameBuffer>> buffers;

		for (uint i = 0; i < cfg.bufferCount; i++)
		{
			std::string name("pica-ros2-" + std::to_string(i));
            size_t fsize_4096 = roundUp4096(cfg.frameSize);
			libcamera::UniqueFD fd = this->dma_heap.alloc(name.c_str(), fsize_4096);

			if (!fd.isValid())
				throw std::runtime_error("Failed to allocate capture buffers for stream");

			std::vector<FrameBuffer::Plane> plane(1);
			plane[0].fd = libcamera::SharedFD(std::move(fd));
			plane[0].offset = 0;
			plane[0].length = fsize_4096;

			buffers.push_back(std::make_unique<FrameBuffer>(plane));
			void *memory = mmap(NULL, fsize_4096, PROT_READ | PROT_WRITE, MAP_SHARED, plane[0].fd.get(), 0);

            uint plane_offset = 0;
            for (uint j = 0; j < 3; j++) {
                
                uint plane_stride = (j == 0) ? this->stride : this->stride / 2;
                uint plane_length = plane_stride * (j == 0 ? this->height : this->height / 2);
                
                this->mapped_buffers[buffers.back().get()].push_back(av_buffer_create(
                    static_cast<uint8_t*>(memory) + plane_offset,
                    plane_length,
                    free_buffer,
                    reinterpret_cast<void*>(fsize_4096),
                    0
                ));
                this->mapped_buffer_strides[buffers.back().get()].push_back(plane_stride);
                plane_offset += plane_length;
            }
            
			// libcamera::Span<uint8_t>(static_cast<uint8_t *>(memory), cfg.frameSize));
		}

        this->frame_buffers[stream] = std::move(buffers);

        std::cout << "Allocated " << this->frame_buffers[stream].size() << " dma buffers for stream pixel format: " << cfg.pixelFormat.toString() << std::endl;

        // int ret = allocator->allocate(stream);
        // if (ret < 0) {
        //     std::cerr << "Can't allocate buffers" << std::endl;
        //     return;
        // }
        // const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
        // size_t allocated = buffers.size();
        // std::cout << "Allocated " << allocated << " buffers for stream pixel format: " << cfg.pixelFormat.toString() << std::endl;

        for (unsigned int i = 0; i < this->frame_buffers[stream].size(); ++i) {
            std::unique_ptr<Request> request = camera->createRequest();
            if (!request)
            {
                std::cerr << "Can't create request" << std::endl;
                return;
            }

            const std::unique_ptr<FrameBuffer> &buffer = this->frame_buffers[stream][i];
            int ret = request->addBuffer(stream, buffer.get());
            if (ret < 0)
            {
                std::cerr << "Can't set buffer for request" << std::endl;
                return;
            }
            this->requests.push_back(std::move(request));
        }
    }

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

    this->frame = av_frame_alloc();
    if (!this->frame){
        std::cerr << "Could not allocate video frame" << std::endl;
        return;
    }

    this->frame->format = this->codec_context->pix_fmt;
    this->frame->height = this->codec_context->height;
    this->frame->width = this->codec_context->width;

    this->packet = av_packet_alloc();
    if (this->packet == NULL) {
        std::cerr << RED << "Error making packet" << CLR << std::endl;
        return;
    }

    std_msgs::msg::Header header;
    header.frame_id = this->frame_id;
    header.stamp = builtin_interfaces::msg::Time();
    this->outFrameMsg.header = header;
    this->outFrameMsg.width = this->codec_context->width;
    this->outFrameMsg.height = this->codec_context->height;
    this->outFrameMsg.encoding = "h.264";
    this->outFrameMsg.is_bigendian = false;

    // if (this->hw_encoder) { // wait a bit to allow encoder init
    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    // }

    this->camera->requestCompleted.connect(this, &CameraInterface::frameRequestComplete);
    this->camera->start();

    for (std::unique_ptr<Request> &request : this->requests) {
        this->camera->queueRequest(request.get());
    }

    // this->eventLoop();
}

void CameraInterface::stop() {
    if (!this->running)
        return;
    this->running = false;
}

void CameraInterface::eventLoop() {
    std::cout << GREEN << "Camera " << this->model << " loop starting..." << CLR << std::endl;
    while (this->running) {
        std::cout << "Ohi " << this->model << std::endl;
        this->lines_printed++;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    std::cout << RED << "Camera " << this->model << " loop stopped" << CLR << std::endl;
    this->camera->stop();
    this->camera->requestCompleted.disconnect(this, &CameraInterface::frameRequestComplete);
    this->camera->release();
    this->camera.reset();
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
    this->codec_context->refs = 5;

    /// Compression efficiency (slower -> better quality + higher cpu%)
    /// [ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow]
    /// Set this option to "ultrafast" is critical for realtime encoding
    av_opt_set(this->codec_context->priv_data, "preset", "ultrafast", 0);
    
    // std::string n_buffs = fmt::format("{}", );
    av_opt_set_int(codec_context->priv_data, "num_capture_buffers", this->buffer_count, 0);
    av_opt_set(codec_context->priv_data, "input-io-mode", "dmabuf", 0);

    /// Compression rate (lower -> higher compression) compress to lower size, makes decoded image more noisy
    /// Range: [0; 51], sane range: [18; 26]. I used 35 as good compression/quality compromise. This option also critical for realtime encoding
    // std::string crf = fmt::format("{}", this->compression);
    av_opt_set_int(this->codec_context->priv_data, "crf", this->compression, 0);

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

void CameraInterface::frameRequestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled)
        return;
    
    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    bool log = false;
    
    if (this->log_message_every_sec > -0.001) {
        std::time_t current_time = std::time(nullptr);
        if (current_time - this->last_log >= this->log_message_every_sec) {
            this->last_log = current_time;
            log = true;
        }
    }

    struct dma_buf_sync dma_sync {};
    dma_sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

		// auto &list = this->frame_buffers[const_cast<Stream *>(bufferPair.first)];
		// auto it = std::find_if(list.begin(), list.end(),
		// 					[&bufferPair] (auto &b) { return b.get() == bufferPair.second;} );
		// if (it == list.end())
		//     throw std::runtime_error("Failed to identify request buffer");
        
        int ret = ::ioctl(bufferPair.second->planes()[0].fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
		if (ret)
		    throw std::runtime_error("Failed to sync dma buf on queue request");

		// if (request->addBuffer(bufferPair.first, bufferPair.second) < 0)
		// 	throw std::runtime_error("Failed to add buffer to request in QueueRequest");

        // const std::vector<libcamera::FrameBuffer::Plane>& planes = buffer->planes();

        // if (!planes[0].fd.isValid()) {
        //     std::cerr << "Frame plane[0] fd invalid" << std::endl;
        //     continue;
        // }

        // size_t total_size = 0;
        // for (size_t i = 0; i < planes.size(); ++i) {
        //     size_t plane_end = planes[i].offset + planes[i].length;
        //     if (plane_end > total_size) {
        //         total_size = plane_end;
        //     }
        // }

        std::time_t current_time = std::time(nullptr);
        if (current_time - this->last_fps_time >= 1.0) {
            //std::time_t d = current_time - this->last_fps;
            this->last_fps = this->frame_count;
            this->last_fps_time = current_time;
            this->frame_count = 0;
        }
        this->frame_count++;

        // unsigned int nplane = 0;
        // std::cout << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence << " bytesused: ";
        // for (const FrameMetadata::Plane &plane : metadata.planes())  {
            // std::cout << plane.bytesused;
            // if (++nplane < metadata.planes().size()) std::cout << "/";
        // }
        // std::cout << std::endl;

        // void* buf_base = mmap(nullptr, total_size, PROT_READ, MAP_SHARED, planes[0].fd.get(), 0);
        // if (buf_base == MAP_FAILED) {
        //     std::cerr << "Failed to mmap entire buffer: " << strerror(errno) << std::endl;
        //     return;
        // }

        if (log) {
            if (!this->log_scrolls && this->lines_printed > 0) {
                for (int i = 0; i < this->lines_printed; i++) {
                    std::cout << std::string(this->lines_printed, '\033') << "[A\033[K";
                }
            }
            this->lines_printed = 0;
        }

        if (log) {
            std::cout << this->last_fps << " FPS" << std::endl;
            this->lines_printed++;

            // std::cout << "Buff " << total_size << " total B" << std::endl;
            // this->lines_printed++;

            std::cout << std::setw(6) << std::setfill('0') << metadata.sequence << ": ";
        }

    
        // int plane_offset = 0;
        auto &list = this->mapped_buffers[bufferPair.second];
        auto &plane_strides = this->mapped_buffer_strides[bufferPair.second];

        for (size_t i = 0; i < 3; ++i) {

            // int plane_stride = (i == 0) ? this->stride : this->stride / 2;
            // int plane_length = plane_stride * (i == 0 ? this->height : this->height / 2);

            // Map the memory
            // if (log) {
            //     if (i == 0)
            //         std::cout << "fd=" << planes[i].fd.get() << "; ";
            //     std::cout << "pl#" << i <<" (" << plane_length << ") " ;
            //     std::cout << plane_offset << "-" << (plane_offset+plane_length) << " ";
            //     std::cout << "|" << plane_stride << "|";
            //     if (i < 2)
            //         std::cout << "; ";
            // }
            
            this->frame->buf[i] = list[i];
            // plane_offset += plane_length;

            // Assign plane data using offsets within the mapped buffer
            this->frame->data[i] = this->frame->buf[i]->data;
            this->frame->linesize[i] = plane_strides[i];
        }
        if (log) {
            std::cout << std::endl;
            this->lines_printed++;
        }

        // continue;

        /// Set frame index in range: [1, fps]
        this->frame->pts = this->frameIdx;

        /// Set frame type
        bool isKeyFrame = false; // this->frameIdx == 0;
        if (isKeyFrame){
            this->frame->key_frame = 1;
            this->frame->pict_type = AVPictureType::AV_PICTURE_TYPE_I;
        }
        // std::cout << "Sending..." << std::endl;

        bool frame_ok = false;
        switch (avcodec_send_frame(this->codec_context, this->frame)){
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

        // if (munmap(buf_base, total_size) == -1) {
        //     std::cerr << "munmap failed: " << strerror(errno) << std::endl;
        // }
        // buf_base = nullptr;  // Optional: prevent accidental reuse

        if (!frame_ok) {
            continue;
        }

        bool packet_ok = false;
        switch (avcodec_receive_packet(this->codec_context, this->packet)) {
            case 0:
                /// use packet, copy/send it's data, or whatever
                packet_ok = true;
                if (log) {
                    std::cout << (this->packet->flags == 1 ? MAGENTA : YELLOW);
                    
                    std::cout << "PACKET " << this->packet->size
                            << " / " << this->packet->buf->size
                            << " pts=" << this->packet->pts
                            << " flags=" << this->packet->flags;
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
            
            uint64_t timestamp_ns = metadata.timestamp;
            get_current_stamp(&this->outFrameMsg.header.stamp, timestamp_ns);
            this->outFrameMsg.pts = av_rescale_q(this->packet->pts /*this->outFrameMsg.header.stamp.sec * NS_TO_SEC + this->outFrameMsg.header.stamp.nanosec*/,
                                           codec_context->time_base,
                                           AVRational{1, 90000});

            this->outFrameMsg.flags = this->packet->flags;
            // outFrameMsg.data = &packet->data; // uint8[] out
            // outFrameMsg.data = std::vector<uint8_t>(packet->size);
            this->outFrameMsg.data.assign(this->packet->data, this->packet->data + this->packet->size);

            if (log) {
                std::cout << GREEN << " >> Sending " << this->outFrameMsg.data.size() << "B" << CLR << " sec: " << this->outFrameMsg.header.stamp.sec << " nsec: " << outFrameMsg.header.stamp.nanosec << std::endl;
                this->lines_printed++;
            }
        
            this->publisher->publish(this->outFrameMsg);
        }
    }

    if (!this->running)
        return;

    request->reuse(Request::ReuseBuffers);
    this->camera->queueRequest(request);
}

void CameraInterface::readConfig() {
    std::string config_prefix = fmt::format("/camera_{}.", this->location);
    
    this->node->declare_parameter(config_prefix + "hflip", false);
    this->node->declare_parameter(config_prefix + "vflip", false);

    this->node->declare_parameter(config_prefix + "hw_encoder", true);
    this->hw_encoder = this->node->get_parameter(config_prefix + "hw_encoder").as_bool();

    this->node->declare_parameter(config_prefix + "width", 1920);
    this->width = (uint) this->node->get_parameter(config_prefix + "width").as_int();
    this->node->declare_parameter(config_prefix + "height", 1080);
    this->height = (uint) this->node->get_parameter(config_prefix + "height").as_int();

    this->node->declare_parameter(config_prefix + "bitrate", 4000000);
    this->bit_rate = this->node->get_parameter(config_prefix + "bitrate").as_int();

    this->node->declare_parameter(config_prefix + "compression", 35);
    this->compression = this->node->get_parameter(config_prefix + "compression").as_int();

    this->node->declare_parameter(config_prefix + "framerate", 30);
    this->fps = this->node->get_parameter(config_prefix + "framerate").as_int();

    this->node->declare_parameter(config_prefix + "buffer_count", 4);
    this->buffer_count = (uint) this->node->get_parameter(config_prefix + "buffer_count").as_int();

    this->node->declare_parameter(config_prefix + "frame_id", "picam");
    this->frame_id = this->node->get_parameter(config_prefix + "frame_id").as_string();

    this->log_scrolls = this->node->get_parameter("log_scroll").as_bool();
    this->log_message_every_sec = this->node->get_parameter("log_message_every_sec").as_double();
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
    std::cout << BLUE << "Cleaning up " << this->model << " interface" << CLR << std::endl;

    // for (auto &iter : this->mapped_buffers)
	// {
	// 	// assert(iter.first->planes().size() == iter.second.size());
	// 	// for (unsigned i = 0; i < iter.first->planes().size(); i++)
	// 	// for (auto &span : iter.second)
	// 	// 	munmap(span.data(), span.size());
	// }
	this->mapped_buffers.clear();
	this->frame_buffers.clear();

    av_frame_unref(this->frame);
    av_frame_free(&this->frame);
    av_packet_unref(this->packet);
    av_packet_free(&this->packet);
    
    this->camera = NULL;
    this->node = NULL;
    avcodec_close(this->codec_context);
    avcodec_free_context(&this->codec_context);
}