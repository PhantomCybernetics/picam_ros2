
#include "picam_ros2/encoder_hw.hpp"
#include "picam_ros2/camera_interface.hpp"

EncoderHW::EncoderHW(CameraInterface *interface, std::shared_ptr<libcamera::Camera> camera)
	: Encoder (interface, camera) {

    std::cout << GREEN << "Using HW encoder" << CLR << std::endl;
    
    const char device_name[] = "/dev/video11";
	this->encoder_fd = open(device_name, O_RDWR, 0);
	if (this->encoder_fd < 0)
		throw std::runtime_error("failed to open V4L2 H264 encoder");
	std::cout << "Opened H264Encoder on " << device_name << " as fd " << this->encoder_fd << std::endl;

	// Apply any options->

	v4l2_control ctrl = {};	
    ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
    ctrl.value = this->interface->bit_rate;
    if (xioctl(this->encoder_fd , VIDIOC_S_CTRL, &ctrl) < 0)
        throw std::runtime_error("failed to set bitrate");
	
    // static const std::map<std::string, int> profile_map =
    //     { { "baseline", V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE },
    //       { "main", V4L2_MPEG_VIDEO_H264_PROFILE_MAIN },
    //       { "high", V4L2_MPEG_VIDEO_H264_PROFILE_HIGH } };
    // auto it = profile_map.find(options->profile);
    // if (it == profile_map.end())
    //     throw std::runtime_error("no such profile " + options->profile);
    ctrl.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
    ctrl.value = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
    if (xioctl(this->encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
        throw std::runtime_error("failed to set profile");
	
    // static const std::map<std::string, int> level_map =
    // 	{ { "4", V4L2_MPEG_VIDEO_H264_LEVEL_4_0 },
    // 	  { "4.1", V4L2_MPEG_VIDEO_H264_LEVEL_4_1 },
    // 	  { "4.2", V4L2_MPEG_VIDEO_H264_LEVEL_4_2 } };
    // auto it = level_map.find(options->level);
    // if (it == level_map.end())
    // 	throw std::runtime_error("no such level " + options->level);
    ctrl.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
    ctrl.value = V4L2_MPEG_VIDEO_H264_LEVEL_4_2;
    if (xioctl(this->encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
        throw std::runtime_error("failed to set level");
	
    ctrl.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD;
    ctrl.value = 0;
    if (xioctl(this->encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
        throw std::runtime_error("failed to set intra period");
	
    ctrl.id = V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER;
    ctrl.value = 1;
    if (xioctl(this->encoder_fd, VIDIOC_S_CTRL, &ctrl) < 0)
        throw std::runtime_error("failed to set inline headers");
	
	// Set the output and capture formats. We know exactly what they will be.

    v4l2_format fmt = {};
	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.width = this->interface->width;
	fmt.fmt.pix_mp.height = this->interface->height;
	// We assume YUV420 here, but it would be nice if we could do something
	// like info.pixel_format.toV4L2Fourcc();
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420;
	fmt.fmt.pix_mp.plane_fmt[0].bytesperline = this->interface->width;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt.fmt.pix_mp.num_planes = 1;
	if (xioctl(this->encoder_fd, VIDIOC_S_FMT, &fmt) < 0)
		throw std::runtime_error("failed to set output format");

	fmt = {};
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	fmt.fmt.pix_mp.width = this->interface->width;
	fmt.fmt.pix_mp.height = this->interface->height;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].bytesperline = 0;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = 512 << 10;
	if (xioctl(this->encoder_fd, VIDIOC_S_FMT, &fmt) < 0)
		throw std::runtime_error("failed to set capture format");

    struct v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    parm.parm.output.timeperframe.numerator = 90000.0 / this->interface->fps;
    parm.parm.output.timeperframe.denominator = 90000;
    if (xioctl(this->encoder_fd, VIDIOC_S_PARM, &parm) < 0)
        throw std::runtime_error("failed to set streamparm");

	// Request that the necessary buffers are allocated. The output queue
	// (input to the encoder) shares buffers from our caller, these must be
	// DMABUFs. Buffers for the encoded bitstream must be allocated and
	// m-mapped.

    v4l2_requestbuffers reqbufs = {};
	reqbufs.count = this->interface->buffer_count;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbufs.memory = V4L2_MEMORY_DMABUF;
	if (xioctl(this->encoder_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		throw std::runtime_error("request for output buffers failed");
	std::cout << "Got " << reqbufs.count << " output buffers" << std::endl;

	// We have to maintain a list of the buffers we can use when our caller gives
	// us another frame to encode.
	// for (unsigned int i = 0; i < reqbufs.count; i++)
	// 	input_buffers_available_.push(i);

	reqbufs = {};
	reqbufs.count = 16;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (xioctl(this->encoder_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		throw std::runtime_error("request for capture buffers failed");
	std::cout << "Got " << reqbufs.count << " capture buffers" << std::endl;
	// num_capture_buffers_ = reqbufs.count;

    this->hw_buffers = new BufferDescription[reqbufs.count]; // CameraInterface::BufferDescription();
	for (uint i = 0; i < reqbufs.count; i++)
	{
		v4l2_plane planes[VIDEO_MAX_PLANES];
		v4l2_buffer buffer = {};
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;
		buffer.length = 1;
		buffer.m.planes = planes;
		if (xioctl(this->encoder_fd, VIDIOC_QUERYBUF, &buffer) < 0)
			throw std::runtime_error("failed to capture query buffer " + std::to_string(i));
		this->hw_buffers[i].mem = mmap(0, buffer.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, this->encoder_fd,
							   buffer.m.planes[0].m.mem_offset);
		if (this->hw_buffers[i].mem == MAP_FAILED)
			throw std::runtime_error("failed to mmap capture buffer " + std::to_string(i));
		this->hw_buffers[i].size = buffer.m.planes[0].length;
		// Whilst we're going through all the capture buffers, we may as well queue
		// them ready for the encoder to write into.
		if (xioctl(this->encoder_fd, VIDIOC_QBUF, &buffer) < 0)
			throw std::runtime_error("failed to queue capture buffer " + std::to_string(i));
	}

	// Enable streaming and we're done.

	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (xioctl(this->encoder_fd, VIDIOC_STREAMON, &type) < 0)
		throw std::runtime_error("failed to start output streaming");
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (xioctl(this->encoder_fd, VIDIOC_STREAMON, &type) < 0)
		throw std::runtime_error("failed to start capture streaming");
	std::cout << "Codec streaming started" << std::endl;

	// output_thread_ = std::thread(&H264Encoder::outputThread, this);
	// poll_thread_ = std::thread(&H264Encoder::pollThread, this);

    std::cerr << CYAN << "Encoder initiated for " << this->interface->width << "x" << this->interface->height << " @ " << this->interface->fps << " fps" << "; BPP=" << this->interface->bytes_per_pixel << CLR << std::endl;
}

void EncoderHW::captureRequestComplete(std::vector<AVBufferRef *> plane_buffers, std::vector<uint> plane_strides, int64_t *frameIdx, long timestamp_ns, bool log) {



}
