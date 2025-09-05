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

#ifndef EIGEN_PUBSUB__PUB_HPP_
#define EIGEN_PUBSUB__PUB_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "rclcpp/executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;  // NOLINT

class EigenPublisher : public rclcpp::Node
{
public:
  EigenPublisher() : rclcpp::Node("eigen_publisher_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/eigen_test", 10);
    auto timer_callback = [this]() -> void { this->timerCallback(); };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  static Eigen::MatrixXf createMatrix()
  {
    Eigen::MatrixXf mat(3, 3);
    mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    return mat;
  }

  void timerCallback()
  {
    auto message = std_msgs::msg::Float32MultiArray();
    Eigen::MatrixXf mat = createMatrix();
    message.layout.dim.resize(2);
    message.layout.dim[0].label = "rows";
    message.layout.dim[0].size = mat.rows();
    message.layout.dim[0].stride = mat.rows() * mat.cols();
    message.layout.dim[1].label = "cols";
    message.layout.dim[1].size = mat.cols();
    message.layout.dim[1].stride = mat.cols();
    message.data.assign(mat.data(), std::next(mat.data(), mat.size()));
    RCLCPP_INFO(this->get_logger(), "Publishing Eigen matrix as Float32MultiArray");
    publisher_->publish(message);
  }
};

#endif  // EIGEN_PUBSUB__PUB_HPP_