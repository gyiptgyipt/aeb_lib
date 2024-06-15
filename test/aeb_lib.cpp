#include "aeb/rom_aeb.hpp"
#include "aeb/laser_angle_filter.hpp"

using namespace std::chrono_literals;

std::string odom_topic = "/diff_cont/odom";
std::string scan_topic = "/scan";
std::string publish_topic = "/aeb/cmd_vel";
std::string aeb_topic = "/aeb_status";
std::string laser_filtered_topic = "/filtered_scan";



rom_dynamics::vechicle::AEB::AEB() : Node("Rom_AEB_Node") {
    if (!this->has_parameter("ttc_final")) {
        this->declare_parameter("ttc_final", 1.0);
    }

    if( !this->has_parameter("min_angle") && !this->has_parameter("max_angle") )
        {
            this->declare_parameter("min_angle", -90.0);
            this->declare_parameter("max_angle", 90.0);
        }

    double min_angle = this->get_parameter("min_angle").as_double();   //laser rays ရဲ့ angle ကို dynamically ချိန်ညှိနိုင်ရန် နှင့် parameter သတ်မှတ်ရလွယ်ကူစေရန်
    double max_angle = this->get_parameter("max_angle").as_double();

    TTC_final_threshold = this->get_parameter("ttc_final").as_double(); // second

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&rom_dynamics::vechicle::AEB::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&rom_dynamics::vechicle::AEB::odom_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(publish_topic, 10);
    brake_pub_ = this->create_publisher<std_msgs::msg::Bool>(aeb_topic, 10); //brake topic 
    laser_filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_filtered_topic, 10);     // laser_angle_filter အတွက် publisher 

    timer_ = this->create_wall_timer(100ms, std::bind(&AEB::timer_callback, this));

    brake_msg_.linear.x = 0.0000;
    brake_msg_.angular.z = 0.0000;

    RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;33mTTC_Final : %.4f\033[1;0m", TTC_final_threshold);

    }


bool rom_dynamics::vechicle::AEB::Rom_aeb(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, nav_msgs::msg::Odometry::ConstSharedPtr odom_msg){

    LaserAngleFilter laser_filter;
    laser_filter.filter_angle(scan_msg);

}


