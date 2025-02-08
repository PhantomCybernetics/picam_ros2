#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <string>
#include <sys/mman.h>

#include "picam_ros2/encoder_hw.hpp"
#include "picam_ros2/camera_interface.hpp"

#include <bitset>
#include <poll.h>

int xioctl(int fd, unsigned long ctl, void *arg)
{
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

EncoderHW::EncoderHW(CameraInterface *interface, std::shared_ptr<libcamera::Camera> camera)
	: Encoder (interface, camera) {

    std::cout << GREEN << "Using HW encoder" << CLR << std::endl;
    
	// this->time_base.num = 1;
	// this->time_base.den = NS_TO_SEC;

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
	
    ctrl.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD; //keyframe generation period
    ctrl.value = this->interface->fps;
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
	for (uint i = 0; i < reqbufs.count; i++)
		this->input_buffers_available.push(i);

	reqbufs = {};
	reqbufs.count = NUM_CAPTURE_BUFFERS;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (xioctl(this->encoder_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		throw std::runtime_error("request for capture buffers failed");
	std::cout << "Got " << reqbufs.count << " capture buffers" << std::endl;
	// num_capture_buffers_ = reqbufs.count;

    this->hw_buffers = new BufferDescription[reqbufs.count]; // CameraInterface::BufferDescription();
	this->buffer_meta = new BufferMeta[reqbufs.count];
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

	this->poll_thread = std::thread(&EncoderHW::pollThread, this);

    std::cerr << CYAN << "Encoder initiated for " << this->interface->width << "x" << this->interface->height << " @ " << this->interface->fps << " fps" << "; BPP=" << this->interface->bytes_per_pixel << CLR << std::endl;
}


void EncoderHW::encode(std::vector<AVBufferRef *>, std::vector<uint>, int base_fd, uint size, int64_t *frameIdx, long timestamp_ns, bool log) {

	int index = 0;
	{
		// We need to find an available output buffer (input to the codec) to
		// "wrap" the DMABUF.
		std::lock_guard<std::mutex> lock(this->input_buffers_available_mutex);
		if (this->input_buffers_available.empty()) {
			this->interface->err("No buffers available to queue codec input");
			return;
		}
			
		index = this->input_buffers_available.front();
		this->input_buffers_available.pop();
	}

	v4l2_buffer buf = {};
	v4l2_plane planes[VIDEO_MAX_PLANES] = {};
	buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	buf.index = index;
	buf.field = V4L2_FIELD_NONE;
	buf.memory = V4L2_MEMORY_DMABUF;
	buf.length = 1;
	buf.timestamp.tv_sec = timestamp_ns / NS_TO_SEC;
	buf.timestamp.tv_usec = timestamp_ns % 1000;
	buf.m.planes = planes;
	buf.m.planes[0].m.fd = base_fd;
	buf.m.planes[0].bytesused = size;
	buf.m.planes[0].length = size;

	this->buffer_meta[index].frame_idx = *frameIdx;
	this->buffer_meta[index].timestamp_ns = timestamp_ns;
	this->buffer_meta[index].log = log;

	*frameIdx = ((*frameIdx) % this->interface->fps) + 1;

	if (xioctl(this->encoder_fd, VIDIOC_QBUF, &buf) < 0)
		throw std::runtime_error("failed to queue input to codec");

	// std::cout << "Sending frame to be hw-encoded (buff " << index << ")" << std::endl;
	// this->interface->lines_printed++;

}

void EncoderHW::pollThread()
{
	while (true)
	{
		pollfd p = {this->encoder_fd, POLLIN, 0};
		int ret = poll(&p, 1, 200);
		{
			std::lock_guard<std::mutex> lock(this->input_buffers_available_mutex);
			if (this->abort_poll && this->input_buffers_available.size() == this->interface->buffer_count)
				break;
		}
		if (ret == -1)
		{
			if (errno == EINTR)
				continue;
			throw std::runtime_error("unexpected errno " + std::to_string(errno) + " from poll");
		}
		if (p.revents & POLLIN)
		{
			v4l2_buffer buf = {};
			v4l2_plane planes[VIDEO_MAX_PLANES] = {};
			buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
			buf.memory = V4L2_MEMORY_DMABUF;
			buf.length = 1;
			buf.m.planes = planes;
			int ret = xioctl(this->encoder_fd, VIDIOC_DQBUF, &buf);
			BufferMeta meta;
			uint output_index;
			if (ret == 0)
			{
				output_index = buf.index;
				meta = this->buffer_meta[output_index];
				// Return this to the caller, first noting that this buffer, identified
				// by its index, is available for queueing up another frame.
				{
					std::lock_guard<std::mutex> lock(this->input_buffers_available_mutex);
					this->input_buffers_available.push(buf.index);
				}
				// input_done_callback_(nullptr);
			}

			buf = {};
			memset(planes, 0, sizeof(planes));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.length = 1;
			buf.m.planes = planes;
			ret = xioctl(this->encoder_fd, VIDIOC_DQBUF, &buf);
			if (ret == 0)
			{
				// We push this encoded buffer to another thread so that our
				// application can take its time with the data without blocking the
				// encode process.
				// int64_t timestamp_ns = (buf.timestamp.tv_sec * NS_TO_SEC) + buf.timestamp.tv_usec * 1000;

				bool log = meta.log;
				bool keyframe = !!(buf.flags & V4L2_BUF_FLAG_KEYFRAME);

				int64_t timestamp_ns = (int64_t) meta.timestamp_ns;
				if (log) {
					auto clr = keyframe ? MAGENTA : YELLOW;
					this->interface->log(clr, "Frame encoded (buff ", output_index, "/", buf.index, "), ts=", timestamp_ns, "; flags=", std::bitset<4>(buf.flags));
				}

				uint64_t pts = av_rescale_q(meta.frame_idx, //this->encoded_packet->pts, //this->outFrameMsg.header.stamp.sec * NS_TO_SEC + this->outFrameMsg.header.stamp.nanosec, //this->packet->pts
                                    		AVRational{1, (int)this->interface->fps},
                                    		AVRational{1, 90000});
				
				this->interface->publishH264((unsigned char*) this->hw_buffers[buf.index].mem, buf.m.planes[0].bytesused, keyframe, pts, timestamp_ns, log);
				
				// OutputItem item = { buffers_[buf.index].mem,
				// 					buf.m.planes[0].bytesused,
				// 					buf.m.planes[0].length,
				// 					buf.index,
				// 					!!(buf.flags & V4L2_BUF_FLAG_KEYFRAME),
				// 					timestamp_us };
				// std::lock_guard<std::mutex> lock(output_mutex_);
				// output_queue_.push(item);
				// output_cond_var_.notify_one();
				auto index = buf.index;
				auto length = buf.length;

				v4l2_buffer buf = {};
				v4l2_plane planes[VIDEO_MAX_PLANES] = {};
				buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
				buf.memory = V4L2_MEMORY_MMAP;
				buf.index = index;
				buf.length = 1;
				buf.m.planes = planes;
				buf.m.planes[0].bytesused = 0;
				buf.m.planes[0].length = length;
				if (xioctl(this->encoder_fd, VIDIOC_QBUF, &buf) < 0)
					throw std::runtime_error("failed to re-queue encoded buffer");
			}
		}
	}
}

EncoderHW::~EncoderHW() {

	std::cout << BLUE << "Cleaning up hw encoder" << CLR << std::endl;

	this->abort_poll = true;
	this->poll_thread.join();

	// Turn off streaming on both the output and capture queues, and "free" the
	// buffers that we requested. The capture ones need to be "munmapped" first.

	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (xioctl(this->encoder_fd, VIDIOC_STREAMOFF, &type) < 0)
		std::cerr << "Failed to stop output streaming" << std::endl;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (xioctl(this->encoder_fd, VIDIOC_STREAMOFF, &type) < 0)
		std::cerr << "Failed to stop capture streaming" << std::endl;

	v4l2_requestbuffers reqbufs = {};
	reqbufs.count = 0;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbufs.memory = V4L2_MEMORY_DMABUF;
	if (xioctl(this->encoder_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		std::cerr << "Request to free output buffers failed" << std::endl;

	for (int i = 0; i < NUM_CAPTURE_BUFFERS; i++)
		if (munmap(this->hw_buffers[i].mem, this->hw_buffers[i].size) < 0)
			std::cerr << "Failed to unmap buffer" << std::endl;
	reqbufs = {};
	reqbufs.count = 0;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (xioctl(this->encoder_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		std::cerr << "Request to free capture buffers failed" << std::endl;

	close(this->encoder_fd);
	std::cout << "H264Encoder closed" << std::endl;
}
