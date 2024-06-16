#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "aeb/aeb_lib.hpp"

using namespace std::chrono_literals;

class LinearPublisher : public rclcpp::Node
{
  public:
    LinearPublisher()
    : Node("linear_publisher"), count_(60)
    {
      if( !this->has_parameter("linear_speed") )
        {
            this->declare_parameter("linear_speed", 1.0);
        }

    
      const sensor_msgs::msg::LaserScan::ConstSharedPtr sub_scan; //add_ver for DEMO 
      bool should_brake_ = false;
       
      

      twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/linear_vel", 10);
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LinearPublisher::laser_callback, this, std::placeholders::_1));


      timer_ = this->create_wall_timer(
      100ms, std::bind(&LinearPublisher::timer_callback, this));

      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }

  private:
    void timer_callback()
    {
      int timer_second = count_/10;

      if(count_ % 10 == 0 && count_ >= 0)
      { // တစက်ကန့် တခါ timer ပြဖို့ ပါ။
        RCLCPP_INFO(this->get_logger(), "Ready to go !: %d", timer_second);
      }
      count_--;

      if(count_ < 0)
      { // ၆ စက်ကန့် ပြီး ရင် မောင်းမယ်။
        double linear_speed = this->get_parameter("linear_speed").as_double();
        
        rom_dynamics::vechicle::AEB aeb;                                   // library test လုပ်ခြင်း
        const sensor_msgs::msg::LaserScan::ConstSharedPtr sub_scan;

        bool brake_status = aeb.rom_aeb(sub_scan); 
        
        if (brake_status = false){
            // ၆ စက်ကန့်ပြီးရင် တစက်ကန့် ကို ၂ ခါ display ပြရန်။
            msg.linear.x = linear_speed;
            twist_publisher_->publish(msg);

        if(count_ % -5 == 0){
          RCLCPP_INFO(rclcpp::get_logger("\033[1;36mLinear Velocity\033[1;0m"), ": \033[1;36m%.4f m/s\033[1;0m", msg.linear.x);
        }
        else{
            
            msg.linear.x = 0.0;
            twist_publisher_ ->publish(msg);
        }

        }    
        
      }
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laser_msg){
        
        const sensor_msgs::msg::LaserScan::ConstSharedPtr sub_scan = laser_msg;

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    int count_;
    geometry_msgs::msg::Twist msg;
    geometry_msgs::msg::Twist::SharedPtr msg_ptr;
    const sensor_msgs::msg::LaserScan::ConstSharedPtr sub_scan;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearPublisher>());
  rclcpp::shutdown();
  return 0;
}