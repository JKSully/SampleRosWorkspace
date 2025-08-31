#ifndef TEST_PACKAGE__PUB_HPP_
#define TEST_PACKAGE__PUB_HPP_

#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;  // NOLINT

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : rclcpp::Node("publisher_node"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Kenny has been punched " + std::to_string(count_++) + " times";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(50ms, timer_callback);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

#endif  // TEST_PACKAGE__PUB_HPP_
