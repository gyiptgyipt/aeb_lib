#ifndef ROM_DYNAMICS_AEB
#define ROM_DYNAMICS_AEB
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "aeb/laser_angle_filter.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>



//  std::string odom_topic; // global variable 
//  std::string scan_topic;
//  std::string publish_topic;
//  std::string aeb_topic;
namespace rom_dynamics
{
    namespace vechicle
    {
class AEB_LIB : public rclcpp::Node {
public:
    AEB_LIB();
    
    // double TTC_final_threshold, min_angle , max_angle;

    bool calculate_aeb(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg , double min_angle , double max_angle , double TTC_final_threshold);


    
    bool should_brake_ = false;

private:
    geometry_msgs::msg::Twist brake_msg_;
    geometry_msgs::msg::Twist input_vel_;

    std_msgs::msg::Bool brake_status_;

    
    double TTC_final_threshold;
    double odom_velocity_x_ = 0.0;
    double odom_velocity_y_ = 0.0;


    LaserAngleFilter laser_filter;
    sensor_msgs::msg::LaserScan::SharedPtr filter_angle(const sensor_msgs::msg::LaserScan::SharedPtr msg ,double min_angle,double max_angle);
   
       
       };

    };

};
#endif // ROM_DYNAMICS_AEB
