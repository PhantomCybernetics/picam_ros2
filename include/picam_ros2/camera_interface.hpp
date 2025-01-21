#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <libcamera/libcamera.h>
#include <libcamera/pixel_format.h>

#include "picam_ros2.hpp"

using namespace libcamera;

class CameraInterface {
    public:
        CameraInterface(std::shared_ptr<Camera> camera, std::shared_ptr<PicamROS2> node);
        ~CameraInterface();

    private:
        std::shared_ptr<libcamera::Camera> camera;
        std::shared_ptr<PicamROS2> node;
        std::vector<std::unique_ptr<Request>> requests;

        void requestComplete(Request *request);
};