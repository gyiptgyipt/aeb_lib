#include "aeb/rom_aeb.hpp"

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

void rom_dynamics::vechicle::AEB::timer_callback() {
    if (should_brake_ == true) { 
        std_msgs::msg::Bool brake_status_;
        brake_status_.data = should_brake_;
        brake_pub_->publish(brake_status_);
        // twist_pub_->publish(brake_msg_);
        RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;31mBreaking\033[1;0m");
    }
    // else { 
    //     RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;32mReleased\033[1;0m"); 
    //     RCLCPP_INFO(rclcpp::get_logger("\033[1;33mAES\033[1;0m"), ": \033[1;33mTTC_Final : %.4f\033[1;0m", TTC_final_threshold);
    // }
}

void rom_dynamics::vechicle::AEB::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // Actual implementation here
    TTC_final_threshold = this->get_parameter("ttc_final").as_double(); // seconds

    double min_angle = this->get_parameter("min_angle").as_double();   //laser rays ရဲ့ angle ကို dynamically ချိန်ညှိနိုင်ရန် နှင့် parameter သတ်မှတ်ရလွယ်ကူစေရန်
    double max_angle = this->get_parameter("max_angle").as_double();

    
    double min_TTC = 1000000.0;
    double velocity_x = this->odom_velocity_x_;

    

    // 180 ပဲယူမယ်။ ဘာလို့ဆို lidar က 360 ိ ဖြစ်နေလို့ပါ။                  // Lidar angle ကို index နည်းလမ်းဖြင့်ယူခြင်း
    // for (unsigned int i = 90; i < 270; i++) 
    // {
        //RCLCPP_INFO(this->get_logger(), "ranges i : %d", i);

        //if (!std::isinf(scan_msg->ranges[i]) && !std::isnan(scan_msg->ranges[i]))  



    auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg);
        filtered_scan->ranges.clear();


    double angle_min = min_angle * M_PI / 180.0;  // -60 degrees in radians
        double angle_max = max_angle * M_PI / 180.0;   // 60 degrees in radians
        double current_angle = scan_msg->angle_min;


    for (const auto &range : scan_msg->ranges)           // laser data အသစ်ဖန်တီးခြင်း
        {
            if (current_angle >= angle_min && current_angle <= angle_max)
            {
                filtered_scan->ranges.push_back(range);
            }
            else
            {
                filtered_scan->ranges.push_back(std::numeric_limits<float>::infinity());
            }
            current_angle += scan_msg->angle_increment;
        }
    laser_filtered_pub_->publish(*filtered_scan);  // laser_angle_filter အတွက် publisher 


    for (std::size_t i = 0; i < scan_msg->ranges.size(); i++) {
        double distance = scan_msg->ranges[i];
        if (std::isnan(distance) || std::isinf(distance) || distance < scan_msg->range_min) {  // containue if laser data were non and inf and <min_range 
            continue;
        }
        
        double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        double distance_derivative = cos(angle) * velocity_x;    
        
        if (distance_derivative > 0 && (distance/distance_derivative) < min_TTC) {
            min_TTC = distance / std::max(distance_derivative, 0.001);
        }
    }
    should_brake_ = (min_TTC <= TTC_final_threshold);
}

void rom_dynamics::vechicle::AEB::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    this->odom_velocity_x_ = odom_msg->twist.twist.linear.x;
    this->odom_velocity_y_ = odom_msg->twist.twist.linear.y;
}