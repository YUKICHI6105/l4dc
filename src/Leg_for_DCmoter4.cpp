#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "can_plugins2/msg/frame.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "../include/can_utils.hpp"


using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class pubsub : public rclcpp::Node
{
  public:
    pubsub() : Node("l4dc4_node"), count_(0)
    {
      publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
      subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&pubsub::joy_callback, this, _1));
    }

  private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    size_t count_;
};

void pubsub::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {  
    RCLCPP_INFO(this->get_logger(), "I heard:");

    if(msg->buttons[2]==1)
    {
      publisher_->publish(get_frame(0x100,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x110,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x120,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x130,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x140,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x200,static_cast<uint8_t>(5)));
      publisher_->publish(get_frame(0x210,static_cast<uint8_t>(5)));
    }

    if(msg->buttons[1]==1)
    {
      publisher_->publish(get_frame(0x100,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x110,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x120,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x130,static_cast<uint8_t>(0)));
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
    //右回転
    publisher_->publish(get_frame(0x141, 6.28f*(y-x+r)));
    publisher_->publish(get_frame(0x111, 6.28f*(-x-y+r)));
    publisher_->publish(get_frame(0x121, 6.28f*(x-y+r)));
    publisher_->publish(get_frame(0x131, 6.28f*(x+y+r)));
    //chatter.publish(get_frame(0x101, x/static_cast<float>(sqrt(2))-y/static_cast<float>(sqrt(2))));
    //100右上、110左上、120左下、130右下

    RCLCPP_INFO(this->get_logger(), "Publishing:");
  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pubsub>());
  rclcpp::shutdown();
  return 0;
}