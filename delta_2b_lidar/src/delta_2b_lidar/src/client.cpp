/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-09-15
*
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

#define RAD2DEG(x) ((x)*180./M_PI)

using std::placeholders::_1;

class DeltaLidarClient : public rclcpp::Node
{
public:
    DeltaLidarClient() : Node("delta_2b_lidar_node_client")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DeltaLidarClient::scanCallback, this, _1));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
    {
        int count = scan->scan_time / scan->time_increment;
        RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
        RCLCPP_INFO(this->get_logger(), "angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
        
        for(int i = 0; i < count; i++) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            RCLCPP_INFO(this->get_logger(), ": [%f, %f]", degree, scan->ranges[i]);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeltaLidarClient>());
    rclcpp::shutdown();
    return 0;
}
