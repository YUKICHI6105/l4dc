#include <rclcpp/rclcpp.hpp>

enum ShirasuMode{
    dis,
    none1,
    none2,
    none3,
    crr,
    vel,
    pos,
};

struct ShirasuLegID{
    uint32_t upperRightID;
    uint32_t upperLeftID;
    uint32_t lowerLeftID;
    uint32_t lowerRightID;
};