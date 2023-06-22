#include <memory>
#include <string>

#include <chrono>
#include <functional>

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
    ShirasuLegID shirasuID;//シラスのID
    uint8_t valveEnableArray[8];//電磁弁のEnable
    uint32_t valveButtonArray[8];
    uint32_t statusArray[8];//電磁弁の状態
    std::string valveModeArray[8];
    uint8_t countvalve0 = 0;
    uint8_t countvalve1 = 0;
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
      this->declare_parameter("shirasuVelButton", 2);
      this->declare_parameter("shirasuDisButton", 1);
      //ツイスト型の調査
      this->declare_parameter("solenoidValveEnable", rclcpp::PARAMETER_INTEGER_ARRAY);
      std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("solenoidValveEnable", valveEnableArray)};
      this->set_parameters(all_new_parameters);

      this->declare_parameter("solenoidValveButton", rclcpp::PARAMETER_INTEGER_ARRAY);//電磁弁ボタンのパラメーター
      std::vector<rclcpp::Parameter> valve_button_parameters{rclcpp::Parameter("solenoidValveButton", valveButtonArray)};
      this->set_parameters(valve_button_parameters);

      this->declare_parameter("solenoidValveMode", rclcpp::PARAMETER_STRING_ARRAY);//電磁弁のトグルモードの選択
      std::vector<rclcpp::Parameter> valve_mode_parameters{rclcpp::Parameter("solenoidValveMode", valveModeArray)};
      this->set_parameters(valve_mode_parameters);
      
      timer_ = this->create_wall_timer(1000ms, std::bind(&pubsub::timer_callback, this));
      timer_callback();
    }
    void timer_callback();
    void shirasuValuePublish(float upperRight,float upperLeft,float lowerLeft,float lowerRight);
    void shirasuModePublish(uint8_t upperRight,uint8_t upperLeft,uint8_t lowerLeft,uint8_t lowerRight);
    void publishValve(uint32_t channel,uint32_t bottom,std::string mode,const sensor_msgs::msg::Joy::SharedPtr msg1);
};

void pubsub::timer_callback()
    {
      maxSpeed = this->get_parameter("maxSpeed").as_double();
      shirasuID.upperRightID = this->get_parameter("upperRight").as_int();
      shirasuID.upperLeftID = this->get_parameter("upperLeft").as_int();
      shirasuID.lowerLeftID = this->get_parameter("lowerLeft").as_int();
      shirasuID.lowerRightID = this->get_parameter("lowerRight").as_int();
      for(int i=0; i<8; i++){
        valveEnableArray[i] = this->get_parameter("solenoidValveEnable").as_integer_array()[i];
        valveButtonArray[i] = this->get_parameter("solenoidValveButton").as_integer_array()[i];
        valveModeArray[i] = this->get_parameter("solenoidValveMode").as_string_array()[i];
      }
      RCLCPP_INFO(this->get_logger(), "max speed %f!", maxSpeed);
      RCLCPP_INFO(this->get_logger(), "upperRight %d! upperLeft %d! lowerLeft %d! lowerRight %d!", shirasuID.upperRightID ,shirasuID.upperLeftID, shirasuID.lowerLeftID, shirasuID.lowerRightID);
      RCLCPP_INFO(this->get_logger(), "valve_mode %d %d %d %d %d %d %d %d!", valveEnableArray[0],valveEnableArray[1],valveEnableArray[2],valveEnableArray[3],valveEnableArray[4],valveEnableArray[5],valveEnableArray[6],valveEnableArray[7]);
      RCLCPP_INFO(this->get_logger(), "solenoidValveButton %d %d %d %d %d %d %d %d!", valveButtonArray[0],valveButtonArray[1],valveButtonArray[2],valveButtonArray[3],valveButtonArray[4],valveButtonArray[5],valveButtonArray[6],valveButtonArray[7]);
      RCLCPP_INFO(this->get_logger(), "solenoidValveMode %s %s %s %s %s %s %s %s!", valveModeArray[0].c_str(), valveModeArray[1].c_str(), valveModeArray[2].c_str(), valveModeArray[3].c_str(), valveModeArray[4].c_str(), valveModeArray[5].c_str(), valveModeArray[6].c_str(), valveModeArray[7].c_str());
    }

void pubsub::shirasuValuePublish(float upperRight,float upperLeft,float lowerLeft,float lowerRight){
  publisher_->publish(shirasu_frame(shirasuID.upperRightID+1, upperRight));
  publisher_->publish(shirasu_frame(shirasuID.upperLeftID+1, upperLeft));
  publisher_->publish(shirasu_frame(shirasuID.lowerLeftID+1, lowerLeft));
  publisher_->publish(shirasu_frame(shirasuID.lowerRightID+1, lowerRight));
  //100右上、110左上、120左下、130右下
}

void pubsub::shirasuModePublish(uint8_t upperRight,uint8_t upperLeft,uint8_t lowerLeft,uint8_t lowerRight){
  publisher_->publish(get_frame(shirasuID.upperRightID, static_cast<uint8_t>(upperRight)));
  publisher_->publish(get_frame(shirasuID.upperLeftID, static_cast<uint8_t>(upperLeft)));
  publisher_->publish(get_frame(shirasuID.lowerLeftID, static_cast<uint8_t>(lowerLeft)));
  publisher_->publish(get_frame(shirasuID.lowerRightID, static_cast<uint8_t>(lowerRight)));
  //100右上、110左上、120左下、130右下
}

void pubsub::publishValve(uint32_t channel,uint32_t button,std::string mode,const sensor_msgs::msg::Joy::SharedPtr msg1){
  if(msg1->buttons[button]==1){
    if(countvalve1==0){
      if(mode == "Toggle"){
        if(statusArray[channel-1]==1){
          statusArray[channel-1]=0;
        }else if(statusArray[channel-1]){
          statusArray[channel-1]=1;
        }
      }else if(mode == "Normal"){
        statusArray[channel-1]=1;
      }
      publisher_->publish(get_frame(0x101,statusArray));
      countvalve1=1;
    }else if(countvalve0==1){
      countvalve0 = 0;
    }
  }else if(msg1->buttons[button]==0){
    if(countvalve0==0){
      if(mode=="Normal"){
        statusArray[channel-1]=0;
        publisher_->publish(get_frame(0x101,statusArray));
        countvalve0=1;     
      }
    }
    if(countvalve1==1){
      countvalve1=0;
    }
  }
}

void pubsub::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  { 
    for(int i=0;i<7;i++){
      if(valveEnableArray[i]==1){
        publishValve(i+1,this->get_parameter("solenoidValveButton").as_integer_array()[i],this->get_parameter("solenoidValveMode").as_string_array()[i],msg);
      }
    }
//    RCLCPP_INFO(this->get_logger(), "I heard:");
    if(msg->buttons[this->get_parameter("shirasuVelButton").as_int()]==1)
    {
      shirasuModePublish(5,5,5,5);
      publisher_->publish(valve_frame(0x100, valveEnableArray));
    }

    if(msg->buttons[this->get_parameter("shirasuDisButton").as_int()]==1)
    {
      shirasuModePublish(0,0,0,0);
      publisher_->publish(get_frame(0x100,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x140,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x200,static_cast<uint8_t>(0)));
      publisher_->publish(get_frame(0x210,static_cast<uint8_t>(0)));
    }

    float x= -(msg->axes[0]);
    float y=  (msg->axes[1]);
    float r= 0;
    if(msg->buttons[4]==1)
    {
      r =1.0f;//↑左回転
    }
    else if(msg->buttons[5]==1)
    {
     r =-1.0f;//右回転
    }
    else if(msg->buttons[4] == msg->buttons[5])
    {
     r =0.0f;
    }
    if((x != 0) || (y != 0)){
      shirasuValuePublish(maxSpeed*(y-x+r),maxSpeed*(-x-y+r),maxSpeed*(x-y+r),maxSpeed*(x+y+r));
      //chatter.publish(get_frame(0x101, x/static_cast<float>(sqrt(2))-y/static_cast<float>(sqrt(2))));
      count = 0;

      RCLCPP_INFO(this->get_logger(), "Publishing:bokuha warukunai!");
      std::string str = std::to_string(1.0f*(y-x+r));
      const char* cstr = str.c_str();
      RCLCPP_INFO(this->get_logger(), cstr);
    }
    else{
      if(count == 0){
        shirasuValuePublish(0.0f,0.0f,0.0f,0.0f);
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