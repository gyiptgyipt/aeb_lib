#include "rclcpp/rclcpp.hpp"
#include "aeb/rom_aeb.hpp"

#//include "aeb/laser_angle_filter.hpp"


int main(int argc, char **argv) {

    auto aeb_node = std::make_shared<rom_dynamics::vechicle::AEB>();

    //auto laser_filter_node = std::make_shared<LaserAngleFilter>();

    rclcpp::init(0, nullptr);   //ရှာဖတ်ရန်
    rclcpp::spin(aeb_node);
    rclcpp::shutdown();
    return 0;
}
