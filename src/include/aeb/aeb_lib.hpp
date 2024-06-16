#ifndef AEB_LIB
#define AEB_LIB
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
class AEB : public rclcpp::Node {
public:
    AEB();
    
    bool rom_aeb(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    

private:

    bool should_brake_ = false;
    double TTC_final_threshold;


    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg_;

    // rclcpp::TimerBase::SharedPtr timer_; 
    
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;     //publishers
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_filtered_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;   // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
   
       
       };

    };

};
#endif // ROM_DYNAMICS_AEB
