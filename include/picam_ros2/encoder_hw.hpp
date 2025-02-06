#pragma once

#include <libavutil/rational.h>
#include <queue>
#include <thread>
#include <mutex>
#include "encoder_base.hpp"

class EncoderHW : public Encoder {
    public:
        EncoderHW(CameraInterface *interface, std::shared_ptr<libcamera::Camera> camera);
        ~EncoderHW();
        void encode(std::vector<AVBufferRef *> plane_buffers, std::vector<uint> plane_strides, int base_fd, uint size, int64_t *frameIdx, long timestamp_ns, bool log);
        
    private:
    	// We want at least as many output buffers as there are in the camera queue
        // (we always want to be able to queue them when they arrive). Make loads
        // of capture buffers, as this is our buffering mechanism in case of delays
        // dealing with the output bitstream.
        // static constexpr int NUM_OUTPUT_BUFFERS = 6;
        static constexpr int NUM_CAPTURE_BUFFERS = 12;

        int encoder_fd;
        struct BufferDescription
        {
            void *mem;
            size_t size;
        };
        BufferDescription *hw_buffers;
        struct BufferMeta {
            int64_t frame_idx;
            long timestamp_ns;
            bool log;
        };
        BufferMeta *buffer_meta;
        std::mutex input_buffers_available_mutex;
        std::queue<int> input_buffers_available;
        void pollThread();
        std::thread poll_thread;

        bool abort_poll = false;
        // AVRational time_base;
};