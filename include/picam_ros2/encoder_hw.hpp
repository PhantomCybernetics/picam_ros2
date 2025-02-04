#pragma once

#include "encoder_base.hpp"

class EncoderHW : Encoder {
    public:
        EncoderHW(CameraInterface *interface, std::shared_ptr<libcamera::Camera> camera);
        ~EncoderHW();
        void captureRequestComplete(std::vector<AVBufferRef *> plane_buffers, std::vector<uint> plane_strides, int64_t *frameIdx, long timestamp_ns, bool log);
        
    private:
        int encoder_fd;
        struct BufferDescription
        {
            void *mem;
            size_t size;
        };
        BufferDescription *hw_buffers;
        
};