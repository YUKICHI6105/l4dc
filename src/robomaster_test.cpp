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
    Robomaster<RobomasterFeedback> feedback;
    float x=0.0; 
    float y=0.0; 
    float r=0.0;
    float roopTime = 0.001;
    
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
    this->declare_parameter("urGainArray", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("ulGainArray", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("llGainArray", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("lrGainArray", rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    void timer_callback();
    void can_callback(const can_plugins2::msg::Frame msg);
    void robomasterValuePublish(float upperRight,float upperLeft,float lowerLeft,float lowerRight);
    float calculatePID(float r,RobomasterFeedback feedback);
};

void RoboMaster::timer_callback(){
    feedback.upperRight.number = this->get_parameter("upperRight").as_int();
    feedback.upperLeft.number = this->get_parameter("upperLeft").as_int();
    feedback.lowerLeft.number = this->get_parameter("lowerLeft").as_int();
    feedback.lowerRight.number = this->get_parameter("lowerRight").as_int();
    feedback.upperRight.KP=this->get_parameter("urGainArray").as_double_array()[0];
    feedback.upperRight.KI=this->get_parameter("urGainArray").as_double_array()[1];
    feedback.upperRight.KD=this->get_parameter("urGainArray").as_double_array()[2];
    feedback.upperLeft.KP=this->get_parameter("ulGainArray").as_double_array()[0];
    feedback.upperLeft.KI=this->get_parameter("ulGainArray").as_double_array()[1];
    feedback.upperLeft.KD=this->get_parameter("ulGainArray").as_double_array()[2];
    feedback.lowerLeft.KP=this->get_parameter("llGainArray").as_double_array()[0];
    feedback.lowerLeft.KI=this->get_parameter("llGainArray").as_double_array()[1];
    feedback.lowerLeft.KD=this->get_parameter("llGainArray").as_double_array()[2];
    feedback.lowerRight.KP=this->get_parameter("lrGainArray").as_double_array()[0];
    feedback.lowerRight.KI=this->get_parameter("lrGainArray").as_double_array()[1];
    feedback.lowerRight.KD=this->get_parameter("lrGainArray").as_double_array()[2];
    robomasterValuePublish(feedback.upperRight.target,feedback.upperLeft.target,feedback.lowerLeft.target,feedback.lowerRight.target);
}

void RoboMaster::can_callback(const can_plugins2::msg::Frame msg){
    if(msg.id == (0x200+feedback.upperRight.number)){
        feedback.upperRight.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.upperRight.speed = combineBytesToFloat(msg.data[2],msg.data[3])/60;
        feedback.upperRight.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.upperRight.temperature = msg.data[6];
        feedback.upperRight.target=feedback.upperRight.target+calculatePID((y-x+r),feedback.upperRight);
        RCLCPP_INFO(this->get_logger(),"ID %u current %u",msg.id,feedback.upperRight.current);
    }else if(msg.id == (0x200+feedback.upperLeft.number)){
        feedback.upperLeft.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.upperLeft.speed = combineBytesToFloat(msg.data[2],msg.data[3])/60;
        feedback.upperLeft.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.upperLeft.temperature = msg.data[6];
        feedback.upperLeft.target = feedback.upperLeft.target + calculatePID((-x-y+r),feedback.upperRight);
        RCLCPP_INFO(this->get_logger(),"ID %u current %u",msg.id,feedback.upperLeft.current);
    }else if(msg.id == (0x200+feedback.lowerLeft.number)){
        feedback.lowerLeft.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.lowerLeft.speed = combineBytesToFloat(msg.data[2],msg.data[3])/60;
        feedback.lowerLeft.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.lowerLeft.temperature = msg.data[6];
        feedback.lowerLeft.target = feedback.lowerLeft.target + calculatePID((x-y+r),feedback.lowerLeft);
        RCLCPP_INFO(this->get_logger(),"ID %u current %u",msg.id,feedback.lowerLeft.current);
    }else if(msg.id == (0x200+feedback.lowerRight.number)){
        feedback.lowerRight.locate = combineBytes(msg.data[0],msg.data[1]);
        feedback.lowerRight.speed = combineBytesToFloat(msg.data[2],msg.data[3])/60;
        feedback.lowerRight.current = combineBytes(msg.data[4],msg.data[5]);
        feedback.lowerRight.temperature = msg.data[6];
        feedback.lowerRight.target = feedback.lowerRight.target + calculatePID((x+y+r),feedback.lowerRight);
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

float RoboMaster::calculatePID(float r,RobomasterFeedback feedback){
    // 下記変数は既に与えられているものとする
    // y       : 現在の出力
    // r       : 現在の目標値
    // e_pre   : 前回の誤差
    // T       : 制御周期
    // KP,KI,KD: P,I,Dゲイン

  // PID制御の式より、制御入力uを計算
  float e  = r - feedback.speed;                // 誤差を計算
  float de = (e - feedback.e_pre)/roopTime;        // 誤差の微分を近似計算
  feedback.ie = feedback.ie + (e + feedback.e_pre)*roopTime/2;       // 誤差の積分を近似計算
  feedback.e_pre = e;
  float u  = feedback.KP*e + feedback.KI*feedback.ie + feedback.KD*de; // PID制御の式にそれぞれを代入
  return u;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboMaster>());
    rclcpp::shutdown();
    return 0;
}