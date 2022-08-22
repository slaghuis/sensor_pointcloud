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

/* ************************************************************************************
 * SensorPrecipitator Class
 * Purpose: Class that converts 'range msg' and 'tranforms' into PointCloud2
 *          Also publishes sensor frame transforms if defined
 * @Author Eric Slaghuis (github @slaghuis)
 * @version 1.1 (3/4/2022)
 * 
 * Adapted from the work of 
 * @author Eliot Lim (github: @eliotlim)
 * @version 1.0 (16/5/17)
 * 
 * Adapted from http://docs.ros.org/hydro/api/segbot_sensors/html/range__to__cloud_8cpp_source.html
 *   
 * ************************************************************************************/

#include <sensor_pointcloud/sensor_precipitator.h>

SensorPrecipitator::SensorPrecipitator(rclcpp::Node::SharedPtr node, std::string pointcloud_topic, std::string map_frame) :
                                     node_(node), frame_(map_frame) {
                                       
  // ROS Setup
  pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 3);
                                       
  // Transform listener
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 
  // Start publisher thread
  float rate = 20.0;
  std::thread{std::bind(&SensorPrecipitator::execute, this, std::placeholders::_1), rate}.detach();                                     
}

std::shared_ptr<Sensor> SensorPrecipitator::add_sensor(std::string topic, std::string frame) {
  auto sensor_ptr = std::shared_ptr<Sensor>(new Sensor(node_, topic, frame));
  sensors.push_back(sensor_ptr);
  RCLCPP_INFO(node_->get_logger(), "Inserted Sensor - Topic: %s, Frame: %s", topic.c_str(), frame.c_str());
  return sensor_ptr;
}



void SensorPrecipitator::execute(double rate) {
  rclcpp::Rate loop_rate(rate);
                                       
  while (rclcpp::ok()) {
    
    // Fill point cloud
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = node_->get_clock()->now();
    msg.height = 1;
    msg.width = sensors.size(); //Number of sensors
    
    msg.point_step = 12; // x,y,z as float32 = 3*4bytes
    msg.row_step = 24;  // width * point_step - rather odd, cause it is redundant
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::msg::PointField::FLOAT32);  
  
    sensor_msgs::PointCloud2Iterator<float> out_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(msg, "z");  

    // Flag to check if any data was added to the cloud
    bool data_added = false;
     
    // Convert all Sensor readings to Points
    for (std::vector<std::shared_ptr<Sensor>>::iterator sensorIt = sensors.begin(); sensorIt != sensors.end(); ++sensorIt) { 
        std::shared_ptr<Sensor> sensor = *sensorIt;
      
      auto range = sensor->get_range();
      
      // Check Sensor Range Validity
      if (range < 0) { continue; }
      
      // Get StampedTransform for Sensor
      geometry_msgs::msg::TransformStamped transform;
      try {
        // Calling lookupTransform with tf2::TimePointZero
        // results in the latest available transform
        transform = tf_buffer_->lookupTransform(frame_,
                                                sensor->frame_,
                                                tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s : %s", 
                    sensor->frame_.c_str(), frame_.c_str(), ex.what());
        continue; // skip this reading
      }
            
      // Lookup the transformed coordinates of this point
      geometry_msgs::msg::PointStamped sensor_point;
      sensor_point.header.frame_id = sensor->frame_;

      //we'll just use the most recent transform available for our simple example
     // sensor_point.header.stamp = node_->get_clock()->now();

      //just an arbitrary point in space
      sensor_point.point.x = range;
      sensor_point.point.y = 0.0;
      sensor_point.point.z = 0.0;

      geometry_msgs::msg::PointStamped base_point;
      tf2::doTransform(sensor_point, base_point, transform);
      
      // Store the point in the pointcloud
      *out_x = base_point.point.x;
      *out_y = base_point.point.y;
      *out_z = base_point.point.z;      
      
      ++out_x;
      ++out_y;
      ++out_z;
      
      data_added = true;
    }
    
    if(data_added) {
      pointcloud_publisher_->publish(msg);
      RCLCPP_DEBUG(node_->get_logger(), "PointCloud published");
    } 
    
    loop_rate.sleep();
  }
}
