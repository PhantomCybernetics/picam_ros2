#pragma once

#include "const.hpp"
#include <libcamera/libcamera.h>
// #include <libcamera/pixel_format.h>
// #include <linux/videodev2.h>
// #include <fmt/core.h>
// #include <linux/videodev2.h>
// #include <fmt/core.h>

extern "C" {
    #include <libavutil/buffer.h>
}

class CameraInterface;

class Encoder {
    public:
        Encoder(CameraInterface *interface, std::shared_ptr<libcamera::Camera> camera);
        virtual ~Encoder();
        virtual void encode(std::vector<AVBufferRef *> plane_buffers, std::vector<uint> plane_strides, int base_fd, uint size, int64_t *frameIdx, long timestamp_ns, bool log) = 0;

    protected:
        CameraInterface * interface;
        std::shared_ptr<libcamera::Camera> camera;

};