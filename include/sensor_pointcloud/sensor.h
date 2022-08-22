// Copyright (c) 2021, 2022 Eric Slaghuis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "sensor_msgs/msg/range.hpp"

//using std::placeholders::_1;

class Sensor {
public:
  rclcpp::Node::SharedPtr node_;         
  std::string topic_;
  std::string frame_;
    
  Sensor(rclcpp::Node::SharedPtr node, std::string topic, std::string frame);

  void set_transform(const geometry_msgs::msg::TransformStamped transformS);
  float get_range();

private:
  
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_;
  std::shared_ptr<sensor_msgs::msg::Range> range_msg;
  
  void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);
};