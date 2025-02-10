#include <chrono>
#include <functional>
#include <filesystem>
#include <memory>
#include <string>
#include <future>
#include <vector>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "libcamera/libcamera.h"
#include <linux/videodev2.h>

#include "picam_ros2/picam_ros2.hpp"
#include "picam_ros2/camera_interface.hpp"
#include "picam_ros2/const.hpp"

using namespace libcamera;
using namespace std::chrono_literals;

PicamROS2::PicamROS2(std::string node_name) : Node(node_name), count_(0)
{   
    this->declare_parameter("topic_prefix", "/picam_ros2/camera_");
    this->declare_parameter("log_message_every_sec", 5.0); // -1.0 = off
    this->declare_parameter("log_scroll", false);
    this->declare_parameter("calibration_frames_needed", 10);
    this->declare_parameter("calibration_pattern_size", std::vector<int>{ 9, 6 });
    this->declare_parameter("calibration_square_size_m", 0.019f);
    this->declare_parameter("calibration_files", "/calibration/");
}

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

int main(int argc, char * argv[])
{
    reloadUdevRules();

    checkCameraStack();
    std::cout << "Cam stack ok" << std::endl; 

    auto cm = std::make_unique<CameraManager>();
    cm->start();
    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cout << "No cameras were found on the system." << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    std::string node_name = "picam_ros2";
    YAML::Node node_config = YAML::LoadFile("/ros2_ws/picam_ros2_params.yaml");
    if (node_config["/**"]["ros__parameters"]["node_name"].IsDefined()) {
        node_name = node_config["/**"]["ros__parameters"]["node_name"].as<std::string>();
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicamROS2>(node_name);

    std::vector<std::shared_ptr<CameraInterface>> camera_interfaces;

    for (auto const &c : cameras) {
        
        auto camera = cm->get(c->id());

        int location = -1, rotation = -1;
        std::string model = "N/A";

        const ControlList &props = camera->properties();
        
        std::string searchStr = "i2c@"; // the location returned by props.get(properties::Location).value() is always 2, so parsing the id string
        size_t pos = c->id().find(searchStr);
        if (pos != std::string::npos) {
            size_t startPos = pos + searchStr.length();
            size_t endPos = c->id().find('/', startPos);
            if (endPos != std::string::npos) {
                std::string numberStr = c->id().substr(startPos, endPos - startPos);
                location = std::stoi(numberStr);
            }
        }

        if (props.contains(properties::Model.id())) {
            model = props.get(properties::Model)->c_str();
        }
        if (props.contains(properties::Rotation.id())) {
            rotation = props.get(properties::Rotation).value();
        }  

        std::cout << CYAN << "Found Cam ID=" << c->id() << ", Location=" << location << ", Rotation=" << rotation << "; Model=" << model << CLR << std::endl;
        
        if (location == -1) {
            std::cerr << RED << "Cam ID=" << c->id() << ": error parsing location; camera will be disabled" << CLR << std::endl;
            continue;
        }

        auto config_prefix = CameraInterface::GetConfigPrefix(location);

        node->declare_parameter(config_prefix + "enabled", true); // on by default
        auto enabled = node->get_parameter(config_prefix + "enabled").as_bool();
        if (!enabled) {
            std::cout << "Cam ID=" << c->id() << " at location=" << location << " disabled by config" << std::endl;
            continue;
        }

        auto cam_interface = std::make_shared<CameraInterface>(camera, location, rotation, model, node);
        camera_interfaces.push_back(cam_interface);
        cam_interface->start();
    }

    rclcpp::spin(node);
    
    std::cout << "Yo, shutting down..." << std::endl;

    for (uint i = 0; i < camera_interfaces.size(); i++) {
        camera_interfaces[i]->stop();
    }

    camera_interfaces.clear();

    cm->stop();
    rclcpp::shutdown();
    return 0;
}