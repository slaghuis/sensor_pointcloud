#ifndef SONAR_PRECIPITATOR_H
#define SONAR_PRECIPITATOR_H

#include <chrono>
#include <memory>
#include <thread>

#include "sensor_pointcloud/sensor.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class SensorPrecipitator {
public:
  SensorPrecipitator(rclcpp::Node::SharedPtr node, const std::string pointcloud_topc, const std::string map_frame);
  ~SensorPrecipitator() {}
  std::shared_ptr<Sensor> add_sensor(const std::string sensor_topic, const std::string sensor_frame);
  
private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    std::string frame_;
        
    void execute(double rate);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;

    std::vector<std::shared_ptr<Sensor> > sensors;
     
};

#endif
