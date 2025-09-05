//  Copyright 2025
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

#ifndef EIGEN_PUBSUB__SUB_HPP_
#define EIGEN_PUBSUB__SUB_HPP_

#include <memory>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class EigenSubscriber : public rclcpp::Node
{
public:
  EigenSubscriber() : rclcpp::Node("eigen_subscriber_node")
  {
    auto callback = [this](const std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void {
      this->subscriptionCallback(msg);
    };
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/eigen_test", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

  void subscriptionCallback(const std_msgs::msg::Float32MultiArray::UniquePtr& msg)
  {
    if (msg->layout.dim.size() != 2)
    {
      RCLCPP_ERROR(this->get_logger(), "Received message does not have 2 dimensions");
      return;
    }
    uint32_t rows = msg->layout.dim[0].size;
    uint32_t cols = msg->layout.dim[1].size;
    uint32_t dim = rows * cols;
    if (static_cast<uint32_t>(msg->data.size()) != dim)
    {
      RCLCPP_ERROR(this->get_logger(), "Data size does not match layout dimensions: expected %i, got %zu", dim,
                   msg->data.size());
      return;
    }
    Eigen::Map<const Eigen::MatrixXf> mat(msg->data.data(), rows, cols);
    std::stringstream ss;
    ss << mat.format(Eigen::IOFormat(Eigen::FullPrecision));
    RCLCPP_INFO(this->get_logger(), "Received matrix:\n%s", ss.str().c_str());
  }
};

#endif  // EIGEN_PUBSUB__SUB_HPP_