// Copyright (c) 2022 Eric Slaghuis
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

/* ********************************************************************************
 *   Sensor Class
 *   Purpose: Class that subscribes to `range_msg`
 *   
 *   @Author Eric Slaghuis (github @slaghuis)
 *   @version 1.0 (3/4/2022)
 * 
 *   Adapted from the work of 
 *   @author Eliot Lim (github: @eliotlim)
 *   @version 1.0 (16/5/17)
 * ********************************************************************************/

#include <sensor_pointcloud/sensor.h>

Sensor::Sensor(rclcpp::Node::SharedPtr node, std::string topic, std::string frame) :
             node_(node), topic_(topic), frame_(frame) {
               
    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);           
               
    subscription_ = node_->create_subscription<sensor_msgs::msg::Range>(
      topic_, 10, std::bind(&Sensor::range_callback, this, std::placeholders::_1));           

}

void Sensor::set_transform(geometry_msgs::msg::TransformStamped t) {
  t.child_frame_id = frame_;
  tf_publisher_->sendTransform(t);
}

void Sensor::range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  // Check if frame matches range_msg frame_id
  if (frame_.compare(msg->header.frame_id) == 0) {
    this->range_msg = msg;
  }
}

float Sensor::get_range() {
  // Check range_msg not NULL and min_range/max_range bounds not exceeded
  if (!range_msg) return -1;
  if (range_msg->range < range_msg->min_range || range_msg->range > range_msg->max_range) {
    return -1;
  }
  return range_msg->range;
}