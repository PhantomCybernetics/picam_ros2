#include <sys/mman.h>
#include <sys/ioctl.h>

#include "picam_ros2/const.hpp"
#include "picam_ros2/camera_interface.hpp"
#include "picam_ros2/calibration.hpp"

using namespace libcamera;

void freeBuffer(void* opaque, uint8_t* data) {
    munmap(data, static_cast<size_t>(reinterpret_cast<uintptr_t>(opaque)));
}

uint32_t roundUp4096(uint32_t x) {
    constexpr uint32_t mask = 4096 - 1; // 0xFFF
    return (x + mask) & ~mask;
}

CameraInterface::CameraInterface(std::shared_ptr<Camera> camera, int location, int rotation, std::string model, std::shared_ptr<PicamROS2> node) {
    this->camera = camera;
    this->node = node;
    this->location = location;
    this->rotation = rotation;
    this->model = model;
}

void CameraInterface::start() {
    if (this->running)
        return;

    this->running = true;

    this->log(GREEN, "Initializing ", camera->id());

    // inspect camera

    this->camera->acquire();

    // declare params & read configs
    this->readConfig();
    
    libcamera::Transform transform = Transform::Identity;
	if (this->hflip)
		transform = Transform::HFlip * transform;
	if (this->vflip)
		transform = Transform::VFlip * transform;
    
    // configure camera
    std::unique_ptr<CameraConfiguration> config = this->camera->generateConfiguration( { StreamRole::VideoRecording } );
    
    this->streamConfig = &config->at(0);
    // this->streamConfig->controls.set(libcamera::controls::AeEnable, false);
    this->streamConfig->size.width = this->width;
    this->streamConfig->size.height = this->height;
    this->streamConfig->bufferCount = this->buffer_count;
    config->orientation = config->orientation * transform;
    // this->streamConfig.stride = (uint)this->width;

    if (config->validate() == CameraConfiguration::Invalid)
	    throw std::runtime_error("Failed to validate stream configurations");

    this->stride = this->streamConfig->stride;

    if (this->publish_h264)
        this->log(GREEN, "Publishing as H.264 topic: ", this->h264_topic); 
    else
        this->log(BLUE, "Not publishing as H.264 topic"); 
    
    if (this->publish_image)
        this->log(GREEN, "Publishing as Image topic (", IMAGE_OUTPUT_FORMAT_NAMES.at(this->image_output_format), "): ", this->image_topic); 
    else
        this->log(BLUE, "Not publishing as Image topic"); 

    if (this->publish_info)
        this->log(GREEN, "Publishing CameraInfo: ", this->info_topic);
    else
        this->log(BLUE, "Not publishing CameraInfo");

    this->log(YELLOW, "Camera orinetation: ", config->orientation); 
    this->log(YELLOW, "Stream config: ", this->streamConfig->toString()); 
    this->log(YELLOW, "Stride: ", this->streamConfig->stride); 
    this->log(YELLOW, "Bit rate: ", this->bit_rate); 
    this->log(YELLOW, "Compression rate: ", this->compression); 
    this->log(YELLOW, "Buffer count: ", this->streamConfig->bufferCount); 
    this->log(YELLOW, "Auto exposure enabled: ", this->ae_enable); 
    this->log(YELLOW, "Exposure time: ", this->exposure_time, " ns"); 
    this->log(YELLOW, "Analogue gain: ", this->analog_gain); 
    this->log(YELLOW, "Auto white balance enabled: ", this->awb_enable); 
    this->log(YELLOW, "Auto white balance mode: ", this->awb_mode); 
    // this->log(YELLOW, "Auto white balance locked: ", this->awb_locked); 
    //this->log(YELLOW, "Color gains: {", this->color_gains[0], ", ", this->color_gains[1], "}"); 
    this->log(YELLOW, "Brightness: ", this->brightness); 
    this->log(YELLOW, "Contrast: ", this->contrast); 
    this->camera->configure(config.get());

    // FrameBufferAllocator *allocator = new FrameBufferAllocator(this->camera);

    this->log("Allocating...");
    for (StreamConfiguration &cfg : *config) {
        auto stream = cfg.stream();

        std::vector<std::unique_ptr<FrameBuffer>> buffers;
        this->buffer_size = roundUp4096(cfg.frameSize);

		for (uint i = 0; i < cfg.bufferCount; i++)
		{
			std::string name("pica-ros2-" + std::to_string(i));
			libcamera::UniqueFD fd = this->dma_heap.alloc(name.c_str(), this->buffer_size);

			if (!fd.isValid())
				throw std::runtime_error("Failed to allocate capture buffers for stream");

			std::vector<FrameBuffer::Plane> plane(1);
			plane[0].fd = libcamera::SharedFD(std::move(fd));
			plane[0].offset = 0;
			plane[0].length = this->buffer_size;

			buffers.push_back(std::make_unique<FrameBuffer>(plane));
			void *memory = mmap(NULL, this->buffer_size, PROT_READ , MAP_SHARED, plane[0].fd.get(), 0);

            uint plane_offset = 0;
            for (uint j = 0; j < 3; j++) {
                
                uint plane_stride = (j == 0) ? this->stride : this->stride / 2;
                uint plane_length = plane_stride * (j == 0 ? this->height : this->height / 2);
                
                this->mapped_capture_buffers[buffers.back().get()].push_back(av_buffer_create(
                    static_cast<uint8_t*>(memory) + plane_offset,
                    plane_length,
                    freeBuffer,
                    reinterpret_cast<void*>(this->buffer_size),
                    0
                ));
                this->mapped_capture_buffer_strides[buffers.back().get()].push_back(plane_stride);
                plane_offset += plane_length;
            }
            
			// libcamera::Span<uint8_t>(static_cast<uint8_t *>(memory), cfg.frameSize));
		}

        this->capture_frame_buffers[stream] = std::move(buffers);

        this->log("Allocated ", this->capture_frame_buffers[stream].size(), " capture dma buffers for stream pixel format: ", cfg.pixelFormat.toString());

        // make requests
        for (unsigned int i = 0; i < this->capture_frame_buffers[stream].size(); ++i) {
            std::unique_ptr<Request> request = camera->createRequest();
            if (!request)
            {
                std::cerr << "Can't create request" << std::endl;
                return;
            }

            request->controls().set(libcamera::controls::AeEnable, this->ae_enable);
            if (this->ae_enable) {
                request->controls().set(libcamera::controls::AeMeteringMode, this->ae_metering_mode);
                request->controls().set(libcamera::controls::AeConstraintMode, this->ae_constraint_mode);
                request->controls().set(libcamera::controls::AeExposureMode, this->ae_exposure_mode);
                int64_t frameDurationMax = 1000000 / this->fps; // fps in microseconds
                int64_t frameDurationMin = 1000000 / 60; // fps in microseconds
                request->controls().set(libcamera::controls::FrameDurationLimits, {frameDurationMin, frameDurationMax});
                // request->controls().set(libcamera::controls::ExposureTimeMode, libcamera::controls::ExposureTimeModeAuto);
                // request->controls().set(libcamera::controls::AnalogueGainMode, libcamera::controls::AnalogueGainModeAuto);
            } else if (this->exposure_time > 0.0f) {
                request->controls().set(libcamera::controls::ExposureTime, this->exposure_time);
            }
            
            request->controls().set(libcamera::controls::AnalogueGain, this->analog_gain);
                
            request->controls().set(libcamera::controls::AwbEnable, this->awb_enable);
            if (this->awb_enable) {
                request->controls().set(libcamera::controls::AwbMode, this->awb_mode);
                // request->controls().set(libcamera::controls::AwbLocked, this->awb_locked);
            }
            
            //Span<const float, 2> color_gains({(float)this->color_gains[0], (float)this->color_gains[1]});
            //request->controls().set(libcamera::controls::ColourGains, color_gains);
            request->controls().set(libcamera::controls::Brightness, this->brightness);
            request->controls().set(libcamera::controls::Contrast, this->contrast);

            const std::unique_ptr<FrameBuffer> &buffer = this->capture_frame_buffers[stream][i];
            int ret = request->addBuffer(stream, buffer.get());
            if (ret < 0)
            {
                std::cerr << "Can't set buffer for request" << std::endl;
                return;
            }
            this->capture_requests.push_back(std::move(request));
        }

        this->lines_printed = 0;
    }

    // init ros frame publisher
    
    if (this->publish_h264) {
        this->log("Creating H.264 publisher for ", this->h264_topic);
        auto h264_qos = rclcpp::QoS(1);
        h264_qos.best_effort();
        h264_qos.durability_volatile();
        this->h264_publisher = this->node->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(this->h264_topic, h264_qos);
        
        this->out_h264_msg.header.frame_id = this->frame_id;
        this->out_h264_msg.width = this->width;
        this->out_h264_msg.height = this->height;
        this->out_h264_msg.encoding = "h.264";
        this->out_h264_msg.is_bigendian = false;
    }

    if (this->publish_image) {

        this->log("Creating Image publisher for ", this->image_topic);
        auto image_qos = rclcpp::QoS(1);
        image_qos.best_effort();
        image_qos.durability_volatile();
        this->image_publisher = this->node->create_publisher<sensor_msgs::msg::Image>(this->image_topic, image_qos);
        
        this->out_image_msg.header.frame_id = this->frame_id;
        this->out_image_msg.width = this->width;
        this->out_image_msg.height = this->height;
        this->out_image_msg.encoding = IMAGE_OUTPUT_FORMAT_NAMES.at(this->image_output_format);
        this->out_image_msg.is_bigendian = false;
    }

    if (this->publish_info) {
        this->log("Creating CameraInfo publisher for ", this->info_topic);
        auto info_qos = rclcpp::QoS(1);
        info_qos.best_effort();
        info_qos.durability_volatile();
        this->info_publisher = this->node->create_publisher<sensor_msgs::msg::CameraInfo>(this->info_topic, info_qos);

        this->out_info_msg.header.frame_id = this->frame_id;
        this->out_info_msg.width = this->width;
        this->out_info_msg.height = this->height;

        if (readCalibration(this->calibration_file, this->out_info_msg, this->model, this->width, this->height)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded calibration file %s", this->calibration_file.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load calibration file %s; camera uncalibrated", this->calibration_file.c_str());
        }
    }

    if (this->enable_calibration) {
        this->srv_calibration_toggle = this->node->create_service<std_srvs::srv::SetBool>(fmt::format("camera_{}/calibrate", this->location),
                                                                                          std::bind(&CameraInterface::calibration_toggle, this, std::placeholders::_1, std::placeholders::_2));
        this->srv_calibration_sample_frame = this->node->create_service<std_srvs::srv::Trigger>(fmt::format("camera_{}/sample_frame", this->location),
                                                                                                std::bind(&CameraInterface::calibration_sample_frame, this, std::placeholders::_1, std::placeholders::_2));
        this->srv_calibration_save = this->node->create_service<std_srvs::srv::Trigger>(fmt::format("camera_{}/save_calibration", this->location),
                                                                                        std::bind(&CameraInterface::calibration_save, this, std::placeholders::_1, std::placeholders::_2));
    }

    // std::shared_ptr<CameraInterface> sharedPtr(this, [](CameraInterface* ptr) {
    //     // Custom deleter that does nothing
    // });

    // create the encoder
    if (this->publish_h264) {
        if (this->hw_encoder) {
            this->encoder = (Encoder *) new EncoderHW(this, this->camera);
        } else {
            this->encoder = (Encoder *) new EncoderLibAV(this, this->camera);
        }
    }

    this->camera->requestCompleted.connect(this, &CameraInterface::captureRequestComplete);
    this->camera->start();

    for (std::unique_ptr<Request> &request : this->capture_requests) {
        this->camera->queueRequest(request.get());
    }
}

void CameraInterface::captureRequestComplete(Request *request) {
    if (!this->running || request->status() == Request::RequestCancelled) {
        return;
    }

    const std::map<const Stream *, FrameBuffer *> &request_buffers = request->buffers();

    struct dma_buf_sync dma_sync_start {};
    dma_sync_start.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
    struct dma_buf_sync dma_sync_end {};
    dma_sync_end.flags = DMA_BUF_SYNC_END| DMA_BUF_SYNC_READ;

    for (auto p : request_buffers) {
        FrameBuffer *request_buffer = p.second;
        const FrameMetadata &metadata = request_buffer->metadata();

        auto base_fd = p.second->planes()[0].fd.get(); //base plane

        int ret_start = ::ioctl(base_fd, DMA_BUF_IOCTL_SYNC, &dma_sync_start);
		if (ret_start)
		    throw std::runtime_error("Failed to sync/start dma buf on queue request");
        
        bool log = false;
        auto now = std::chrono::high_resolution_clock::now();
        auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()
        ).count();

        if (this->log_message_every_ns > -1) {        
            if (ns_since_epoch - this->last_log >= this->log_message_every_ns) {
                this->last_log = ns_since_epoch;
                log = true;
            }
        }

        std::time_t current_time = std::time(nullptr);
        if (current_time - this->last_fps_time >= 1.0) {
            this->last_fps = this->frame_count;
            this->last_fps_time = current_time;
            this->frame_count = 0;
        }
        this->frame_count++;

        if (log) {
            // erase latest log lines if not scrolling
            if (!this->log_scrolls && this->lines_printed > 0) {
                for (int i = 0; i < this->lines_printed; i++) {
                    std::cout << std::string(this->lines_printed, '\033') << "[A\033[K";
                }
            }
            this->lines_printed = 0;
        }

        if (log) {
            this->log(this->last_fps, " FPS");
            std::cout << std::setw(6) << std::setfill('0') << metadata.sequence << ": ";
        }

        // int plane_offset = 0;
        auto &plane_buffers = this->mapped_capture_buffers[p.second];
        auto &plane_strides = this->mapped_capture_buffer_strides[p.second];

        int ret_end = ::ioctl(base_fd, DMA_BUF_IOCTL_SYNC, &dma_sync_end);
        if (ret_end)
            throw std::runtime_error("Failed to sync/end dma buf on queue request");
        
        // long timestamp_ns = metadata.timestamp;
        long timestamp_ns = ns_since_epoch;
        if (this->timestamp_ns_base == 0) {
            this->timestamp_ns_base = timestamp_ns;
        }
        timestamp_ns -= this->timestamp_ns_base;

        if (log) {
            std::cout << std::endl;
            this->lines_printed++;
        }

        // encode and publish h.264
        if (this->publish_h264) {
            this->encoder->encode(plane_buffers, plane_strides, base_fd, this->buffer_size, &this->frame_idx, timestamp_ns, log);
        }

        // publish image
        if (this->publish_image) {
            this->publishImage(plane_buffers, plane_strides, this->buffer_size, timestamp_ns, log);
        }

        // publish camera info
        if (this->publish_info) {
            this->publishCameraInfo(timestamp_ns, log);
        }

        // calibration frame capture & handling
        if (this->calibration_running
            && this->calibration_frames.size() < this->calibration_frames_requested
            && ns_since_epoch-last_calibration_frame_taken_ns > calibration_min_frame_delay_ns)
        {
            last_calibration_frame_taken_ns = ns_since_epoch;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sCapturing frame #%lu from %s at location %d%s", GREEN.c_str(), this->calibration_frames.size(), this->model.c_str(), this->location, CLR.c_str());
            this->lines_printed = -1;
            this->calibration_frames.push_back(yuv420ToMonoCopy(plane_buffers, plane_strides, this->width, this->height));
            //cv::imwrite(fmt::format("/ros2_ws/img_snaps/frame_mono_{}.png", ns_since_epoch), this->calibration_frames.back());

            if (this->calibration_frames.size() == this->calibration_frames_needed) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sProcessing calibration  for %s at location %d%s", MAGENTA.c_str(), this->model.c_str(), this->location, CLR.c_str());
                calibrateCamera(this->calibration_frames, this->calibration_pattern_size, this->calibration_square_size, this->out_info_msg);
                this->calibration_running = false;
                this->calibration_frames.clear();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sCalibration complete for %s at location %d%s", MAGENTA.c_str(), this->model.c_str(), this->location, CLR.c_str());
                this->lines_printed = -1;
            }
        }
    }

    if (!this->running)
        return;

    request->reuse(Request::ReuseBuffers);
    this->camera->queueRequest(request);
}


void setCurrentStamp(builtin_interfaces::msg::Time *stamp, uint64_t timestamp_ns) {
    // Split into seconds and nanoseconds
    stamp->sec = static_cast<int32_t>(timestamp_ns / NS_TO_SEC);
    stamp->nanosec = static_cast<uint32_t>(timestamp_ns % NS_TO_SEC);;
}

void CameraInterface::publishH264(unsigned char *data, int size, bool keyframe, uint64_t pts, long timestamp_ns, bool log) {

    setCurrentStamp(&this->out_h264_msg.header.stamp, timestamp_ns);
    this->out_h264_msg.pts = pts;

    this->out_h264_msg.flags = keyframe ? 1 : 0;
    this->out_h264_msg.data.assign(data, data + size);

    if (log) {
        this->log(GREEN, " >> Sending H.264 ", this->out_h264_msg.data.size(), "B", CLR, " sec: ", this->out_h264_msg.header.stamp.sec, " nsec: ", out_h264_msg.header.stamp.nanosec);
    }

    if (rclcpp::ok()) {
        this->h264_publisher->publish(this->out_h264_msg);
    }
}

void CameraInterface::publishImage(const std::vector<AVBufferRef *>& planes, const std::vector<unsigned int>& strides, uint buffer_size, long timestamp_ns, bool log) {

    setCurrentStamp(&this->out_image_msg.header.stamp, timestamp_ns);
    
    switch (this->image_output_format) {
        case IMAGE_OUTPUT_FORMAT::BGR8: // we need to resize U and V planes, which costs CPU time
            {
                // Create Y plane Mat with stride
                cv::Mat y_full(height, strides[0], CV_8UC1, planes[0]->data);

                // Create U and V plane Mats with stride, but only every other row
                cv::Mat u_full(height / 2, strides[1], CV_8UC1, planes[1]->data);
                cv::Mat v_full(height / 2, strides[2], CV_8UC1, planes[2]->data);

                // Resize U and V to match the full image dimensions
                cv::Mat u_resized, v_resized;
                cv::resize(u_full.clone(), u_resized, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
                cv::resize(v_full.clone(), v_resized, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);

                // Combine YUV planes
                std::vector<cv::Mat> yuv_channels = {y_full, u_resized, v_resized};
                cv::Mat yuv;
                cv::merge(yuv_channels, yuv);

                // Convert YUV420 to RGB
                cv::Mat bgr;
                cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR);

                this->out_image_msg.data.assign(bgr.data, bgr.data + (bgr.rows*bgr.cols*3));
            }
            break;
        case IMAGE_OUTPUT_FORMAT::MONO8:
            {
                this->out_image_msg.data.assign(planes[0]->data, planes[0]->data + (this->width*this->height));
            }
            break;
        case IMAGE_OUTPUT_FORMAT::YUV420:
            {
                this->out_image_msg.data.assign(planes[0]->data, planes[0]->data + buffer_size);
            }
            break;
        default:
            return;
    }

    if (log) {
        this->log(GREEN, " >> Sending Image ", this->out_image_msg.data.size(), "B", CLR, " sec: ", this->out_image_msg.header.stamp.sec, " nsec: ", out_image_msg.header.stamp.nanosec);
    }

    if (rclcpp::ok()) {
        this->image_publisher->publish(this->out_image_msg);
    }
}

void CameraInterface::publishCameraInfo(long timestamp_ns, bool log) {

    setCurrentStamp(&this->out_info_msg.header.stamp, timestamp_ns);

    if (log) {
        this->log(GREEN, " >> Sending CameraInfo", CLR, " sec: ", this->out_info_msg.header.stamp.sec, " nsec: ", out_info_msg.header.stamp.nanosec);
    }

    if (rclcpp::ok()) {
        this->info_publisher->publish(this->out_info_msg);
    }
}

void CameraInterface::stop() {
    if (!this->running)
        return;
    this->running = false;
}

void CameraInterface::readConfig() {
    
    auto config_prefix = CameraInterface::GetConfigPrefix(this->location);

    this->node->declare_parameter(config_prefix + "publish_h264", true);
    this->publish_h264 = this->node->get_parameter(config_prefix + "publish_h264").as_bool();

    this->node->declare_parameter(config_prefix + "publish_image", false);
    this->publish_image = this->node->get_parameter(config_prefix + "publish_image").as_bool();

    this->node->declare_parameter(config_prefix + "image_output_format", "yuv420");
    auto image_output_format = this->node->get_parameter(config_prefix + "image_output_format").as_string();
    if (image_output_format == IMAGE_OUTPUT_FORMAT_NAMES.at(IMAGE_OUTPUT_FORMAT::BGR8)) {
        this->image_output_format = IMAGE_OUTPUT_FORMAT::BGR8;
    } else
    if (image_output_format == IMAGE_OUTPUT_FORMAT_NAMES.at(IMAGE_OUTPUT_FORMAT::YUV420)) {
        this->image_output_format = IMAGE_OUTPUT_FORMAT::YUV420;
    } else if (image_output_format == IMAGE_OUTPUT_FORMAT_NAMES.at(IMAGE_OUTPUT_FORMAT::MONO8)) {
        this->image_output_format = IMAGE_OUTPUT_FORMAT::MONO8;
    } else {
        throw std::runtime_error("Invalid image output format, use 'yuv420', 'mono8' or 'bgr8'");
    }

    this->node->declare_parameter(config_prefix + "publish_info", true);
    this->publish_info = this->node->get_parameter(config_prefix + "publish_info").as_bool();

    this->h264_topic = fmt::format(this->node->get_parameter("topic_prefix").as_string() + "{}/{}_h264", this->location, this->model);
    this->image_topic = fmt::format(this->node->get_parameter("topic_prefix").as_string() + "{}/{}", this->location, this->model);
    this->info_topic = fmt::format(this->node->get_parameter("topic_prefix").as_string() + "{}/{}_camera_info", this->location, this->model);

    this->node->declare_parameter(config_prefix + "enable_calibration", true);
    this->enable_calibration = this->node->get_parameter(config_prefix + "enable_calibration").as_bool();


    calibration_frames_needed = (uint) this->node->get_parameter("calibration_frames_needed").as_int();
    auto calibration_pattern_size = this->node->get_parameter("calibration_pattern_size").as_integer_array();
    this->calibration_pattern_size = { (int)calibration_pattern_size[0], (int)calibration_pattern_size[1] };
    this->calibration_square_size = this->node->get_parameter("calibration_square_size_m").as_double();

    this->calibration_files_base_path = this->node->get_parameter("calibration_files").as_string();
    if (!this->calibration_files_base_path.empty() && this->calibration_files_base_path.back() != '/') {
        this->calibration_files_base_path += '/';
    }
    this->calibration_file = fmt::format("{}{}-{}.json", this->calibration_files_base_path, this->model, this->location);

    this->node->declare_parameter(config_prefix + "hflip", false);
    this->node->declare_parameter(config_prefix + "vflip", false);
    this->hflip = this->node->get_parameter(config_prefix + "hflip").as_bool();
    this->vflip = this->node->get_parameter(config_prefix + "vflip").as_bool();

    this->node->declare_parameter(config_prefix + "hw_encoder", true);
    this->hw_encoder = this->node->get_parameter(config_prefix + "hw_encoder").as_bool();

    this->node->declare_parameter(config_prefix + "width", 1920);
    this->width = (uint) this->node->get_parameter(config_prefix + "width").as_int();
    this->node->declare_parameter(config_prefix + "height", 1080);
    this->height = (uint) this->node->get_parameter(config_prefix + "height").as_int();

    this->node->declare_parameter(config_prefix + "bitrate", 4000000);
    this->bit_rate = this->node->get_parameter(config_prefix + "bitrate").as_int();

    this->node->declare_parameter(config_prefix + "ae_enable", true);
    this->ae_enable = this->node->get_parameter(config_prefix + "ae_enable").as_bool();
    this->node->declare_parameter(config_prefix + "exposure_time_ns", 30000); // 10 ms, -1 = off
    this->exposure_time = (uint) this->node->get_parameter(config_prefix + "exposure_time_ns").as_int();
    this->node->declare_parameter(config_prefix + "ae_metering_mode", 0);
    this->ae_metering_mode = (uint) this->node->get_parameter(config_prefix + "ae_metering_mode").as_int();
    // MeteringCentreWeighted = 0,
	// MeteringSpot = 1,
	// MeteringMatrix = 2,
	// MeteringCustom = 3,

    this->node->declare_parameter(config_prefix + "ae_exposure_mode", 0);
    this->ae_exposure_mode = (uint) this->node->get_parameter(config_prefix + "ae_exposure_mode").as_int();
    // ExposureNormal = 0,
	// ExposureShort = 1,
	// ExposureLong = 2,
	// ExposureCustom = 3,

    this->node->declare_parameter(config_prefix + "ae_constraint_mode", 0);
    this->ae_constraint_mode = (uint) this->node->get_parameter(config_prefix + "ae_constraint_mode").as_int();
    // ConstraintNormal = 0,
	// ConstraintHighlight = 1,
	// ConstraintShadows = 2,
	// ConstraintCustom = 3,

    // this->node->declare_parameter(config_prefix + "ae_constraint_mode_values", std::vector<double>{ 2.0f, 1.8f });
    // this->ae_constraint_mode_values = this->node->get_parameter(config_prefix + "ae_constraint_mode_values").as_double_array();

    this->node->declare_parameter(config_prefix + "analog_gain", 1.0); // sensor gain
    this->analog_gain = this->node->get_parameter(config_prefix + "analog_gain").as_double();
    this->node->declare_parameter(config_prefix + "awb_enable", true);
    this->awb_enable = this->node->get_parameter(config_prefix + "awb_enable").as_bool();
    // this->node->declare_parameter(config_prefix + "awb_locked", false);
    // this->awb_locked = this->node->get_parameter(config_prefix + "awb_locked").as_bool();
    this->node->declare_parameter(config_prefix + "awb_mode", 0);
    this->awb_mode = (uint) this->node->get_parameter(config_prefix + "awb_mode").as_int();
    // AwbAuto = 0,
	// AwbIncandescent = 1,
	// AwbTungsten = 2,
	// AwbFluorescent = 3,
	// AwbIndoor = 4,
	// AwbDaylight = 5,
	// AwbCloudy = 6,
	// AwbCustom = 7,

    // this->node->declare_parameter(config_prefix + "color_gains", std::vector<double>{ 2.0f, 1.8f });
    // this->color_gains = this->node->get_parameter(config_prefix + "color_gains").as_double_array();
    
    this->node->declare_parameter(config_prefix + "brightness", 0.2f);
    this->brightness = this->node->get_parameter(config_prefix + "brightness").as_double();
    this->node->declare_parameter(config_prefix + "contrast", 1.2f);
    this->contrast = this->node->get_parameter(config_prefix + "contrast").as_double();

    this->node->declare_parameter(config_prefix + "compression", 35);
    this->compression = (uint) this->node->get_parameter(config_prefix + "compression").as_int();

    this->node->declare_parameter(config_prefix + "framerate", 30);
    this->fps = (uint) this->node->get_parameter(config_prefix + "framerate").as_int();

    this->node->declare_parameter(config_prefix + "buffer_count", 4);
    this->buffer_count = (uint) this->node->get_parameter(config_prefix + "buffer_count").as_int();

    this->node->declare_parameter(config_prefix + "frame_id", "picam");
    this->frame_id = this->node->get_parameter(config_prefix + "frame_id").as_string();

    this->log_scrolls = this->node->get_parameter("log_scroll").as_bool();
    this->log_message_every_ns = (long) (this->node->get_parameter("log_message_every_sec").as_double() * NS_TO_SEC);
}

void CameraInterface::calibration_toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (this->calibration_running && request->data) {
        response->success = false;
        response->message = "Calibration already running";
        return;
    } else if (!this->calibration_running && !request->data) {
        response->success = false;
        response->message = "Calibration not running";
        return;
    }
    this->calibration_running = request->data;

    response->success = true;
    if (this->calibration_running) {
        this->calibration_frames_requested = 0;
        this->calibration_frames.clear();
        this->last_calibration_frame_taken_ns = 0;
        response->message = "Calibration started";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sCalibration started for %s at location %d%s", CYAN.c_str(), this->model.c_str(), this->location, CLR.c_str());
    } else {
        response->message = "Calibration stopped";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sCalibration stopped for %s at location %d%s", RED.c_str(), this->model.c_str(), this->location, CLR.c_str());
    }
    this->lines_printed = -1;
}

void CameraInterface::calibration_sample_frame(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!this->calibration_running) {
        response->success = false;
        response->message = "Calibration not running";
        return;
    }
    if (this->calibration_frames_requested >= this->calibration_frames_needed) {
        response->success = false;
        response->message = "Enough frames collected";
        return;
    }

    this->calibration_frames_requested++;
    
    response->success = true;
    response->message = fmt::format("Capturing frame {} of {}", this->calibration_frames_requested, this->calibration_frames_needed);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%sFrame #%d sampled for %s at location %d%s", CYAN.c_str(), this->calibration_frames_requested, this->model.c_str(), this->location, CLR.c_str());
    this->lines_printed = -1;
    
    if (this->calibration_frames_requested == this->calibration_frames_needed) {
        response->message = fmt::format("Captured {} frames, processing calibration...", this->calibration_frames_requested);
    }
}

void CameraInterface::calibration_save(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (writeCalibration(this->out_info_msg, this->model, this->width, this->height, this->calibration_file)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saved calibration file %s", this->calibration_file.c_str());
        response->success = true;
        response->message = "Calibration saved";
    } else {
        response->success = false;
        response->message = "Failed to save calibration";
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to save calibration in %s", this->calibration_file.c_str());
    }
    
    this->lines_printed = -1;
}

CameraInterface::~CameraInterface() {
    std::cout << BLUE << "Cleaning up " << this->model << " interface" << CLR << std::endl;
    this->running = false;
    this->calibration_running = false;

    try {
        this->camera->stop();
        this->camera->release();
        this->camera->requestCompleted.disconnect(this, &CameraInterface::captureRequestComplete);
        this->camera.reset();

        // for (auto &iter : this->mapped_capture_buffers)
        // {
        // 	assert(iter.first->planes().size() == iter.second.size());
        // 	for (unsigned i = 0; i < iter.first->planes().size(); i++)
        // 	for (auto &span : iter.second)
        // 		munmap(span.data(), span.size());
        // }
        this->mapped_capture_buffers.clear();
        this->capture_frame_buffers.clear();
    } catch (...) {
        std::cout << "Error cleaning up interface" << std::endl;
    }
    
    delete this->encoder;
    this->calibration_frames.clear();
    this->camera = NULL;
    this->node = NULL;
}