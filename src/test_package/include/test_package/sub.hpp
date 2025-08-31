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
