#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include "picam_ros2/camera_interface.hpp"

using namespace libcamera;

CameraInterface::CameraInterface(std::shared_ptr<Camera> camera, std::shared_ptr<PicamROS2> node) {
    std::cout << "Initiating " << camera->id() << std::endl; 
    this->camera = camera;
    this->node = node;

    this->camera->acquire();

    std::unique_ptr<CameraConfiguration> config = this->camera->generateConfiguration( { StreamRole::VideoRecording } );
    
    StreamConfiguration &streamConfig = config->at(0);
    std::cout << "Config: " << streamConfig.toString() << std::endl; 
    // streamConfig.pixelFormat = libcamera::formats::
    this->camera->configure(config.get());

    FrameBufferAllocator *allocator = new FrameBufferAllocator(this->camera);

    std::cout << "Allocating" << std::endl;
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
    camera->start();
    for (std::unique_ptr<Request> &request : this->requests)
        camera->queueRequest(request.get());
}

void CameraInterface::requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled)
        return;
    
    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        std::cout << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence << " bytesused: ";

        unsigned int nplane = 0;
        for (const FrameMetadata::Plane &plane : metadata.planes())
        {
            std::cout << plane.bytesused;
            if (++nplane < metadata.planes().size()) std::cout << "/";
        }

        std::cout << std::endl;

        request->reuse(Request::ReuseBuffers);
        this->camera->queueRequest(request);
    }
}

CameraInterface::~CameraInterface() {
    this->camera->release();
    this->camera = NULL;
    this->node = NULL;
}