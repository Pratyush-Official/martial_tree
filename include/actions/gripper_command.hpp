#ifndef GRIPPER_COMMAND_HPP
#define GRIPPER_COMMAND_HPP

#include <cstdint>

enum class GripperCommand : uint8_t
{
    OPEN  = 0x01,
    CLOSE = 0x02,
    MOTORUP = 0X03,
    MOTORDOWN = 0X04
};

#endif