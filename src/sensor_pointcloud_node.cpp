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
    sensor_pointcloud
    Purpose: ROS2 Node for creating and managing sensor_pointcloud instances
    
    @Author Eric Slaghuis (github @slaghuis)
    @version 1.0 (3/4/2022)
  
    Adapted from the work of 
    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (16/5/17)
*/

#include "sensor_pointcloud/sensor_pointcloud.h"
#include "sensor_pointcloud/sensor_precipitator.h"

using namespace std::chrono_literals;

SensorPointcloud::SensorPointcloud()
  : Node("sensor_pointcloud")
  {
    using namespace std::placeholders;
       
    // Read some parameters
    this->declare_parameter<std::string>("pointcloud_frame", "sensor_pase");
    this->declare_parameter<std::string>("pointcloud_topic", "pointcloud");
    this->declare_parameter("sensors");
    
    // Kick off a init routine
    init_timer_ = this->create_wall_timer( 500ms, std::bind(&SensorPointcloud::init, this) );   
  }

void SensorPointcloud::init()
{
  // run this timer only once
  init_timer_->cancel();
  
  // read parameters
  this->get_parameter("pointcloud_frame", pointcloud_frame_);
  this->get_parameter("pointcloud_topic", pointcloud_topic_);
  RCLCPP_INFO(this->get_logger(), "Pointcloud Frame: %s", pointcloud_frame_.c_str());
  
  rclcpp::Parameter sensor_param("sensors", std::vector<std::string>({}));
  this->get_parameter("sensors", sensor_param);
  
  // Create SensorPrecipitator Object
  auto node_ptr = shared_from_this();
  precipitator = std::make_shared<SensorPrecipitator>(node_ptr, pointcloud_topic_, pointcloud_frame_);
  
  sensors = sensor_param.as_string_array(); 
  if (sensors.size() == 0) RCLCPP_WARN(this->get_logger(), "No Sensors Configured");
  
  for (std::vector<std::string>::iterator sensorNameIt = sensors.begin(); sensorNameIt != sensors.end(); ++sensorNameIt) {
    std::string sensor_topic, sensor_frame;
    
    this->declare_parameter<std::string>(*sensorNameIt + ".topic", "error");
    this->get_parameter(*sensorNameIt + ".topic", sensor_topic);

    this->declare_parameter<std::string>(*sensorNameIt + ".transform.frame", "error");
    this->get_parameter(*sensorNameIt + ".transform.frame", sensor_frame);
        
    RCLCPP_INFO(this->get_logger(), "Sensor Parameters Loaded - Topic: %s Frame: %s ", sensor_topic.c_str(), sensor_frame.c_str());
    
    std::shared_ptr<Sensor> s = precipitator->add_sensor(sensor_topic, sensor_frame);
    
    bool loadTransform = false;
    double translation[3] = {0, 0, 0};
    
    // load x,y,z parameters
    for(char c = 'X'; c<='Z'; c++)
    {
      std::string paramStr = *sensorNameIt + ".transform.pos" + c;
      this->declare_parameter<float>(*sensorNameIt + ".transform.pos" + c, 0.0);
      if (this->get_parameter(paramStr, translation[c-'X'])) {
        loadTransform = true;
        RCLCPP_INFO(this->get_logger(), "Loading transform for %s: %f", paramStr.c_str(), translation[c-'X']);
      }
    }
    
    if (loadTransform) {
      float roll, pitch, yaw;
      this->declare_parameter<float>(*sensorNameIt + ".transform/roll", 0.0);
      this->get_parameter(*sensorNameIt + ".transform/roll",  roll);
      this->declare_parameter<float>(*sensorNameIt + ".transform/pitch", 0.0);
      this->get_parameter(*sensorNameIt + ".transform/pitch", pitch);
      this->declare_parameter<float>(*sensorNameIt + ".transform/yaw", 0.0);
      this->get_parameter(*sensorNameIt + ".transform/yaw",   yaw);
      
      // Load role, pitch and yaw from parameters
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      
      rclcpp::Time now = this->get_clock()->now();
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = now;
      t.header.frame_id = pointcloud_frame_;
      //t.child_frame_id = <<<<<<will be set before message sent

      t.transform.translation.x = translation[0];
      t.transform.translation.y = translation[1];
      t.transform.translation.z = translation[2];
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      
      s->set_transform(t);
           
    }
  }  
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPointcloud>());
  rclcpp::shutdown();
  return 0;
}
