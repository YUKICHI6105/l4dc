#include <memory>
#include <string>

#include <chrono>
#include <functional>

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "../include/can_utils.hpp"
#include "../include/robomaster.hpp"
//#include "../include/l4dc4_setting.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RoboMaster : public rclcpp::Node{
    private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_joy;
    rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr subscriber_can_rx;
    size_t count_;
    robomasterNumber robomaster;
    float x=0.3; 
    float y=0.0; 
    float r=0.0;
    
    public:
    RoboMaster() : Node("robomaster_test_node"), count_(0)
    {
    publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
    subscriber_joy = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&RoboMaster::joy_callback, this, _1));
    subscriber_can_rx = this->create_subscription<can_plugins2::msg::Frame>("can_rx", 10, std::bind(&RoboMaster::can_callback, this, _1));
    timer_ = this->create_wall_timer(200ms, std::bind(&RoboMaster::timer_callback, this));
    this->declare_parameter("upperRight", 1);
    this->declare_parameter("upperLeft", 2);
    this->declare_parameter("lowerLeft", 3);
    this->declare_parameter("lowerRight", 4);
    }
    void timer_callback();
    void can_callback(const can_plugins2::msg::Frame msg);
    void robomasterValuePublish(float upperRight,float upperLeft,float lowerLeft,float lowerRight);
};

void RoboMaster::timer_callback(){
    robomaster.upperRight = this->get_parameter("upperRight").as_int();
    robomaster.upperLeft = this->get_parameter("upperLeft").as_int();
    robomaster.lowerLeft = this->get_parameter("lowerLeft").as_int();
    robomaster.lowerRight = this->get_parameter("lowerRight").as_int();
    robomasterValuePublish((y-x+r),(-x-y+r),(x-y+r),(x+y+r));
}

void RoboMaster::can_callback(const can_plugins2::msg::Frame msg){
    uint16_t test = combineBytes(msg.data[0],msg.data[1]);
    RCLCPP_INFO(this->get_logger(),"current %u %u",msg.id,test);
}

void RoboMaster::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    x= -(msg->axes[0]);
    y=  (msg->axes[1]);
    r= 0;
}

void RoboMaster::robomasterValuePublish(float currentUpperRight,float currentUpperLeft,float currentLowerLeft,float currentLowerRight){
    uint8_t value[8];
    //value[0] = 0;
    formatvalue(currentUpperRight,value,robomaster.upperRight);
    formatvalue(currentUpperLeft,value,robomaster.upperLeft);
    formatvalue(currentLowerLeft,value,robomaster.lowerLeft);
    formatvalue(currentLowerRight,value,robomaster.lowerRight);
    //RCLCPP_INFO(this->get_logger(), "max speed %u!", value[0]); 
    publisher_->publish(robomaster_frame(0x200, value));
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboMaster>());
    rclcpp::shutdown();
    return 0;
}