#include <memory>
#include <string>

#include <chrono>
#include <functional>

#include <array>

#include <rclcpp/rclcpp.hpp>
#include "can_plugins2/msg/frame.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "../include/can_utils.hpp"
#include "../include/l4dc4_setting.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class pubsub : public rclcpp::Node
{
  private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    size_t count_;
    int count = 0;
    float maxSpeed = 0.0f;//厳密にはちょっと違う。
    ShirasuLegID shirasuID;
  public:
    pubsub() : Node("l4dc4_node"), count_(0)
    {
      publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
      subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&pubsub::joy_callback, this, _1));
      this->declare_parameter("maxSpeed", 6.28f);
      this->declare_parameter("upperRight", 0x160);
      this->declare_parameter("upperLeft", 0x164);
      this->declare_parameter("lowerLeft", 0x168);
      this->declare_parameter("lowerRight", 0x16c);
      this->declare_parameter("shirasuVel", 2);
      this->declare_parameter("shirasuDis", 1);
      //ツイスト型の調査
      //電磁弁ボタンのパラメーター
      //電磁弁のトグルモード
//      declare_parameter("solenoidValveEnable");
      this->declare_parameter("solenoidValve1", 0);
      this->declare_parameter("solenoidValve2", 0);
      this->declare_parameter("solenoidValve3", 0);
      this->declare_parameter("solenoidValve4", 0);
      this->declare_parameter("solenoidValve5", 0);
      this->declare_parameter("solenoidValve6", 0);
      this->declare_parameter("solenoidValve7", 0);
      this->declare_parameter("solenoidValveMode1", "Toggle");
      this->declare_parameter("solenoidValveMode2", "Toggle");
      this->declare_parameter("solenoidValveMode3", "Toggle");
      this->declare_parameter("solenoidValveMode4", "Toggle");
      this->declare_parameter("solenoidValveMode5", "Toggle");
      this->declare_parameter("solenoidValveMode6", "Normal");
      this->declare_parameter("solenoidValveMode7", "Normal");
      timer_ = this->create_wall_timer(1000ms, std::bind(&pubsub::timer_callback, this));
      timer_callback();
    }

    void timer_callback()
    {
      maxSpeed = this->get_parameter("maxSpeed").as_double();
      shirasuID.upperRightID = this->get_parameter("upperRight").as_int();
      shirasuID.upperLeftID = this->get_parameter("upperLeft").as_int();
      shirasuID.lowerLeftID = this->get_parameter("lowerLeft").as_int();
      shirasuID.lowerRightID = this->get_parameter("lowerRight").as_int();
//      this->set_parameters(rclcpp::Parameter("solenoidValueEnable", std::vector<uint8_t>({0xff, 0x7f})));
      RCLCPP_INFO(this->get_logger(), "max speed %f!", maxSpeed);
      RCLCPP_INFO(this->get_logger(), "upperRight %d!", shirasuID.upperRightID);
      RCLCPP_INFO(this->get_logger(), "upperLeft %d!", shirasuID.upperLeftID);
      RCLCPP_INFO(this->get_logger(), "lowerLeft %d!", shirasuID.lowerLeftID);
      RCLCPP_INFO(this->get_logger(), "lowerRight %d!", shirasuID.lowerRightID);
      //std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
      //this->set_parameters(all_new_parameters);
    }
};

void pubsub::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {  
//    RCLCPP_INFO(this->get_logger(), "I heard:");
    if(msg->buttons[this->get_parameter("shirasuVel").as_int()]==1)
    {
      publisher_->publish(get_frame(shirasuID.upperRightID,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(shirasuID.upperLeftID,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(shirasuID.lowerLeftID,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(shirasuID.lowerRightID,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x180,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x200,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x210,static_cast<uint8_t>(5)));
    }

    if(msg->buttons[this->get_parameter("shirasuDis").as_int()]==1)
    {
      publisher_->publish(get_frame(shirasuID.upperRightID,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(shirasuID.upperLeftID,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(shirasuID.lowerLeftID,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(shirasuID.lowerRightID,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x140,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x200,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x210,static_cast<uint8_t>(0)));
    }

    float x= -(msg->axes[0]);
    float y=  (msg->axes[1]);
    float r= 0;
    if(msg->buttons[4]==1)
    {
      r =1.0f;
    }
    //↑左回転
    else if(msg->buttons[5]==1)
    {
     r =-1.0f;
    }
    else if(msg->buttons[4] == msg->buttons[5])
    {
     r =0.0f;
    }
    if((x != 0) || (y != 0)){
      //右回転
      publisher_->publish(shirasu_frame(shirasuID.upperRightID+1, maxSpeed*(y-x+r)));
      publisher_->publish(shirasu_frame(shirasuID.upperLeftID+1, maxSpeed*(-x-y+r)));
      publisher_->publish(shirasu_frame(shirasuID.lowerLeftID+1, maxSpeed*(x-y+r)));
      publisher_->publish(shirasu_frame(shirasuID.lowerRightID+1, maxSpeed*(x+y+r)));
      //chatter.publish(get_frame(0x101, x/static_cast<float>(sqrt(2))-y/static_cast<float>(sqrt(2))));
      //100右上、110左上、120左下、130右下
      count = 0;

      RCLCPP_INFO(this->get_logger(), "Publishing:bokuha warukunai!");
      std::string str = std::to_string(1.0f*(y-x+r));
      const char* cstr = str.c_str();
      RCLCPP_INFO(this->get_logger(), cstr);
    }
    else{
      if(count == 0){
        publisher_->publish(shirasu_frame(shirasuID.upperRightID+1, 0.0f));
        publisher_->publish(shirasu_frame(shirasuID.upperLeftID+1, 0.0f));
        publisher_->publish(shirasu_frame(shirasuID.lowerLeftID+1, 0.0f));
        publisher_->publish(shirasu_frame(shirasuID.lowerRightID+1, 0.0f));
        count = 1;
      }
    }
  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pubsub>());
  rclcpp::shutdown();
  return 0;
}