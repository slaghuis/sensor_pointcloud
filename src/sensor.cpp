// Copyright (c) 2021 Xeni Robotics
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

/**
    Sensor Class
    Purpose: Class that subscribes to `range_msg`
    
    @Author Eric Slaghuis (github @slaghuis)
    @version 1.0 (3/4/2022)
  
    Adapted from the work of 
    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include <sensor_pointcloud/sensor.h>

Sensor::Sensor(rclcpp::Node::SharedPtr node, std::string topic, std::string frame) :
             node_(node), topic_(topic), frame_(frame) {
               
    subscription_ = node_->create_subscription<sensor_msgs::msg::Range>(
      topic_, 10, std::bind(&Sensor::range_callback, this, _1));           

    transform_ = false;
}


void Sensor::set_transform(geometry_msgs::msg::TransformStamped transformS) {
    transform_ = true;
    this->transformS = std::shared_ptr<geometry_msgs::msg::TransformStamped>(new geometry_msgs::msg::TransformStamped(transformS));
    this->transformS->child_frame_id = frame_;
}

std::shared_ptr<geometry_msgs::msg::TransformStamped> Sensor::get_transform() { return transformS; }

void Sensor::range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    // Check if frame matches range_msg frame_id
    if (frame_.compare(msg->header.frame_id) == 0) {
        this->range_msg = msg;
        
        RCLCPP_DEBUG(node_->get_logger(), "Received range_msg - frame: %s range: %f", frame_.c_str(), range_msg->range);
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
