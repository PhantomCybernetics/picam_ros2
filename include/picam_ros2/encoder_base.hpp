#pragma once

#include "lib.hpp"
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
        ~Encoder();
        virtual void captureRequestComplete(std::vector<AVBufferRef *> plane_buffers, std::vector<uint> plane_strides, int64_t *frameIdx, long timestamp_ns, bool log) = 0;

    protected:
        CameraInterface * interface;
        std::shared_ptr<libcamera::Camera> camera;

};