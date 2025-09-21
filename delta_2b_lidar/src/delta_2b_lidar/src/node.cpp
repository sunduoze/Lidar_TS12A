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
#include <memory>
#include <vector>
#include <limits>

#include "C3iroboticsLidar.h"
#include "../sdk/include/CSerialConnection.h"

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace std;
using namespace everest::hwdrivers;

typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    uint8_t signal;
    float   angle;
    float   distance;
} RslidarDataComplete;

class Delta2BLidarNode : public rclcpp::Node
{
public:
    Delta2BLidarNode() : Node("delta_2b_lidar_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<std::string>("frame_id", "laser");
        
        // Get parameters
        std::string opt_com_path = this->get_parameter("serial_port").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Create publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        
        // Initialize serial connection
        int opt_com_baudrate = 230400;
        serial_connect_.setBaud(opt_com_baudrate);
        serial_connect_.setPort(opt_com_path.c_str());
        
        if(serial_connect_.openSimple())
        {
            RCLCPP_INFO(this->get_logger(), "Open serial port successful!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Open serial port %s failed!", opt_com_path.c_str());
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "3iRoboticsLidar connected");
        
        robotics_lidar_.initilize(&serial_connect_);
        
        // Create timer for the main loop
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(50),
            std::bind(&Delta2BLidarNode::main_loop, this));
    }

private:
    void publish_scan(std::vector<RslidarDataComplete> &nodes,
                      size_t node_count, rclcpp::Time start,
                      double scan_time,
                      float angle_min, float angle_max)
    {
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id_;
        scan_msg->angle_min = angle_min;
        scan_msg->angle_max = angle_max;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (360.0f - 1.0f);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / static_cast<double>(node_count-1);
        scan_msg->range_min = 0.15;
        scan_msg->range_max = 5.0;

        scan_msg->ranges.resize(360, std::numeric_limits<float>::infinity());
        scan_msg->intensities.resize(360, 0.0);

        // Unpack data
        for (size_t i = 0; i < node_count; i++)
        {
            size_t current_angle = floor(nodes[i].angle);
            if(current_angle > 360.0)
            {
                RCLCPP_WARN(this->get_logger(), "Lidar angle is out of range %d", (int)current_angle);
                continue;
            }
            
            float read_value = static_cast<float>(nodes[i].distance);
            if (read_value < scan_msg->range_min || read_value > scan_msg->range_max)
                scan_msg->ranges[360 - 1 - current_angle] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[360 - 1 - current_angle] = read_value;

            float intensities = static_cast<float>(nodes[i].signal);
            scan_msg->intensities[360 - 1 - current_angle] = intensities;
        }

        scan_pub_->publish(std::move(scan_msg));
    }

    void main_loop()
    {
        TLidarGrabResult result = robotics_lidar_.getScanData();
        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            case LIDAR_GRAB_SUCESS:
            {
                TLidarScan lidar_scan = robotics_lidar_.getLidarScan();
                size_t lidar_scan_size = lidar_scan.getSize();
                std::vector<RslidarDataComplete> send_lidar_scan_data;
                send_lidar_scan_data.resize(lidar_scan_size);
                
                for(size_t i = 0; i < lidar_scan_size; i++)
                {
                    RslidarDataComplete one_lidar_data;
                    one_lidar_data.signal = lidar_scan.signal[i];
                    one_lidar_data.angle = lidar_scan.angle[i];
                    one_lidar_data.distance = lidar_scan.distance[i];
                    send_lidar_scan_data[i] = one_lidar_data;
                }

                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);

                auto end_scan_time = this->now();
                double scan_duration = (end_scan_time - start_scan_time_).seconds() * 1e-3;
                
                RCLCPP_INFO(this->get_logger(), "Receive Lidar count %u!", lidar_scan_size);

                // If successful, publish lidar scan
                publish_scan(send_lidar_scan_data, lidar_scan_size,
                             start_scan_time_, scan_duration,
                             angle_min, angle_max);

                start_scan_time_ = end_scan_time;
                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                RCLCPP_INFO(this->get_logger(), "LIDAR_GRAB_ELSE!");
                break;
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    CSerialConnection serial_connect_;
    C3iroboticsLidar robotics_lidar_;
    std::string frame_id_;
    rclcpp::Time start_scan_time_ = this->now();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Delta2BLidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
