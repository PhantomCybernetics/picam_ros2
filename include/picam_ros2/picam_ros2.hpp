#pragma once

#include "rclcpp/rclcpp.hpp"
#include <future>
#include "std_msgs/msg/string.hpp"
#include <memory>

class PicamROS2 : public rclcpp::Node
{
  public:
    PicamROS2(std::string node_name);
    std::future<int> async_function(int x);
    
  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};