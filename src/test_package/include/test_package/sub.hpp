#ifndef TEST_PACKAGE__SUB_HPP_
#define TEST_PACKAGE__SUB_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber() : rclcpp::Node("subscriber_node")
  {
    auto callback = [this](const std_msgs::msg::String::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "Slash heard: '%s'", msg->data.c_str());
    };
    subscription_ = this->create_subscription<std_msgs::msg::String>("/test_topic", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // TEST_PACKAGE__SUB_HPP_
