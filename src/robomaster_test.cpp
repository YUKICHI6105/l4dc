#include <memory>
#include <string>

#include <chrono>
#include <functional>

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "can_utils.hpp"
#include "robomaster.hpp"
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
    Robomaster<RobomasterFeedback> feedback;
    float x=0.0; 
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
    feedback.upperRight.number = this->get_parameter("upperRight").as_int();
    feedback.upperLeft.number = this->get_parameter("upperLeft").as_int();
    feedback.lowerLeft.number = this->get_parameter("lowerLeft").as_int();
    feedback.lowerRight.number = this->get_parameter("lowerRight").as_int();
    robomasterValuePublish((y-x+r),(-x-y+r),(x-y+r),(x+y+r));
}

void RoboMaster::can_callback(const can_plugins2::msg::Frame msg){
    if(msg.id == (0x200+feedback.upperRight.number)){
        feedback.upperRight.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.upperRight.speed = combineBytes(msg.data[2],msg.data[3]);
        feedback.upperRight.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.upperRight.temperature = msg.data[6];
        RCLCPP_INFO(this->get_logger(),"ID %u current %u",msg.id,feedback.upperRight.current);
    }else if(msg.id == (0x200+feedback.upperLeft.number)){
        feedback.upperLeft.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.upperLeft.speed = combineBytes(msg.data[2],msg.data[3]);
        feedback.upperLeft.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.upperLeft.temperature = msg.data[6];
        RCLCPP_INFO(this->get_logger(),"ID %u current %u",msg.id,feedback.upperLeft.current);
    }else if(msg.id == (0x200+feedback.lowerLeft.number)){
        feedback.lowerLeft.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.lowerLeft.speed = combineBytes(msg.data[2],msg.data[3]);
        feedback.lowerLeft.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.lowerLeft.temperature = msg.data[6];
        RCLCPP_INFO(this->get_logger(),"ID %u current %u",msg.id,feedback.lowerLeft.current);
    }else if(msg.id == (0x200+feedback.lowerRight.number)){
        feedback.lowerRight.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.lowerRight.speed = combineBytes(msg.data[2],msg.data[3]);
        feedback.lowerRight.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.lowerRight.temperature = msg.data[6];
        RCLCPP_INFO(this->get_logger(),"ID %u current %u",msg.id,feedback.lowerRight.current);
    }
}
/*
int hirei;//ゲインと比例値をかけたもの
int sekibunn;//ゲインと積分値をかけたもの
int bibunn;//ゲインと微分値をかけたもの

int mokuhyouu;
int gennzai;

void himozawa(){
int tomozawa = hirei+sekibunn+bibunn+gennzai;
}
*/
void RoboMaster::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    x= -(msg->axes[0]);
    y=  (msg->axes[1]);
    r= 0;
}

void RoboMaster::robomasterValuePublish(float currentUpperRight,float currentUpperLeft,float currentLowerLeft,float currentLowerRight){
    uint8_t value[8];
    formatvalue(currentUpperRight,value,feedback.upperRight.number);
    formatvalue(currentUpperLeft,value,feedback.upperLeft.number);
    formatvalue(currentLowerLeft,value,feedback.lowerLeft.number);
    formatvalue(currentLowerRight,value,feedback.lowerRight.number);
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