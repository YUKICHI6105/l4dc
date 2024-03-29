#include <rclcpp/rclcpp.hpp>
#include <array>
#include <can_plugins2/msg/frame.hpp>

struct RobomasterFeedback{
  uint32_t number;
  uint16_t locate;
  uint16_t speed;
  uint16_t current;
  uint16_t temperature;
};

template <typename T>
struct Robomaster{//ロボマスの番号
  T upperRight;//右上
  T upperLeft;//左上
  T lowerLeft;//左下
  T lowerRight;//右下
};

static std::unique_ptr<can_plugins2::msg::Frame> robomaster_frame(const uint16_t id,uint8_t data[8])
{
  auto frame = std::make_unique<can_plugins2::msg::Frame>();
  frame->id = id;
  frame->is_rtr = false;
  frame->is_extended = false;
  frame->is_error = false;
  frame->dlc = 8;
  for(int i=0;i<8;i++){
    frame->data[i] = data[i];
  }
  
  return frame;
}

void splitUint16(uint16_t input, uint8_t* outputArray,int number) {
  // 上位ビットを取得して下位8ビットに格納
  outputArray[(number-1)*2] = input >> 8;

  // 下位ビットをビットマスクして上位8ビットに格納
  outputArray[(number-1)*2+1] = input & 0xFF;
}

uint16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
  uint16_t result = (static_cast<uint16_t>(highByte) << 8) | lowByte;
  return result;
}

void formatvalue(float current,uint8_t value[8], int number){
  if(current < 0){
    float uncurrent = -current;
    uint16_t uint16current = (uncurrent/20)*16384;
    uint16current =~ uint16current ;
    splitUint16(uint16current,value,number);
  }else{
    uint16_t uint16current = (current/20)*16384;
    splitUint16(uint16current,value,number);
  }
}