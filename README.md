# Sensor PointCloud
A ROS2 package that contains a single node to read `sensor_msgs::msg::range` topics and combine the measurements into a `sensor_msgs::msg::PointCloud2` message.  

## Overview
Range sensors are defined in a YAML configuration file.  This node subscribes to each of theses sensors and publishes a static transform for each sensor (based on where they are mounted on the body of the robot.  This node teads these static transforms and transforms all the measurements into a single measurement, that is published at a set frequency.

## Use Case
Where the robot is equopped with multiple sensors, that typically have to be included into a map server to feed the [ROS2 Nav2](https://navigation.ros.org) stack.  This is a good alternative to a scanning Lidar.

## References
A ROS2 implementation of the work by Eliot Lim titled [sensor_pointcloud](https://github.com/eliotlim/sensor_pointcloud) initially published under the MIT licnese.
