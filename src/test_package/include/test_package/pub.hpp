//  Copyright 2025 author
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

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
