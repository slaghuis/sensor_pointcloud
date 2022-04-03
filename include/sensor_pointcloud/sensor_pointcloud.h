#ifndef SENSOR_POINTCLOUD_H
#define SENSOR_POINTCLOUD_H

#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_pointcloud/sensor_precipitator.h"

class SensorPointcloud : public rclcpp::Node
{
  public:
   SensorPointcloud();

  private:
    // Parameters
    std::string pointcloud_frame_;
    std::string pointcloud_topic_;    
    std::vector<std::string> sensors = {};
    
    std::shared_ptr<SensorPrecipitator> precipitator;
    
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    void init();
    
};


#endif
