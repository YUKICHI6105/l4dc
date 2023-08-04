#include <rclcpp/rclcpp.hpp>
#include <array>
#include <can_plugins2/msg/frame.hpp>
#define CAN_MTU 8

enum ShirasuMode{
  dis,
  none1,
  none2,
  none3,
  crr,
  vel,
  pos,
};

struct ShirasuLegID{//シラスのID
  uint32_t upperRightID;//右上
  uint32_t upperLeftID;//左上
  uint32_t lowerLeftID;//左下
  uint32_t lowerRightID;//右下
  uint32_t syoukouID;
};

struct Valve{
  uint8_t enable[7];//電磁弁のEnable
  uint32_t status[7];//電磁弁の状態
  std::string mode[7];//ToggleかNomalか
  uint32_t button[7];//ボタン割り当て
  uint32_t transmit = 0;
  uint8_t countvalve1[7];
  uint32_t preButton[7];
};
/*
static std::unique_ptr<can_plugins2::msg::Frame> valve_frame(const uint16_t id,uint8_t data[7])
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
*/