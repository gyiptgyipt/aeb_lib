#ifndef LASER_ANGLE_FILTER
#define LASER_ANGLE_FILTER

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>



//  std::string odom_topic; // global variable 
//  std::string scan_topic;
//  std::string publish_topic;
//  std::string aeb_topic;

class LaserAngleFilter : public rclcpp::Node {
public:

    LaserAngleFilter();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    
    double min_angle;
    double max_angle;
    double current_angle;

    void filter_angle(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:

    


};


#endif // Laser_angle_Filter
