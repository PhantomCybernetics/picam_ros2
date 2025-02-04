#include "picam_ros2/lib.hpp"

void reloadUdevRules() {
    if (!std::filesystem::exists("/.phntm_devices_initialized")) {
        std::cout << "\033[35mFirst run, initializing udev rules for /dev\033[0m" << std::endl;        
        std::system("/ros2_ws/src/picam_ros2/scripts/reload-devices.sh");
        std::cout << "\033[35mUdev rules initialized\033[0m" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));  // needs a bit for the udev rules to take effect and picam init sucessfuly
    }
}

void checkCameraStack()
{
	int fd = open("/dev/video0", O_RDWR, 0);
	if (fd < 0)
		return;

	v4l2_capability caps;
	unsigned long request = VIDIOC_QUERYCAP;

	int ret = ioctl(fd, request, &caps);
	close(fd);

	if (ret < 0 || strcmp((char *)caps.driver, "bm2835 mmal"))
		return;

	std::cerr << "ERROR: the system appears to be configured for the legacy camera stack" << std::endl;
	exit(-1);
}

void getCurrentStamp(builtin_interfaces::msg::Time *stamp, uint64_t timestamp_ns) {
    // Split into seconds and nanoseconds
    stamp->sec = static_cast<int32_t>(timestamp_ns / NS_TO_SEC);
    stamp->nanosec = static_cast<uint32_t>(timestamp_ns % NS_TO_SEC);;
}

void freeBuffer(void* opaque, uint8_t* data) {
    munmap(data, static_cast<size_t>(reinterpret_cast<uintptr_t>(opaque)));
}

int xioctl(int fd, unsigned long ctl, void *arg)
{
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

// static int get_v4l2_colorspace(std::optional<libcamera::ColorSpace> const &cs)
// {
// 	if (cs == libcamera::ColorSpace::Rec709)
// 		return V4L2_COLORSPACE_REC709;
// 	else if (cs == libcamera::ColorSpace::Smpte170m)
// 		return V4L2_COLORSPACE_SMPTE170M;

// 	LOG(1, "H264: surprising colour space: " << libcamera::ColorSpace::toString(cs));
// 	return V4L2_COLORSPACE_SMPTE170M;
// }

uint32_t roundUp4096(uint32_t x) {
    constexpr uint32_t mask = 4096 - 1; // 0xFFF
    return (x + mask) & ~mask;
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