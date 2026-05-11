#ifndef YAW_SETPOINT_HPP
#define YAW_SETPOINT_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "ar_interfaces/action/movement.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "qos_profiles.hpp"
#include <cmath> 

#include "ament_index_cpp/get_package_share_directory.hpp"

enum CurrentYaw{
    FRONT = 0,
    RIGHT = 1,
    BACK = 2,
    LEFT = 3,
};

namespace BT {
    template <>
    inline CurrentYaw convertFromString(StringView str) {
        if      (str == "FRONT") return CurrentYaw::FRONT;
        else if (str == "RIGHT") return CurrentYaw::RIGHT;
        else if (str == "BACK")  return CurrentYaw::BACK;
        else if (str == "LEFT")  return CurrentYaw::LEFT;
        throw RuntimeError("Invalid CurrentYaw string: ", str);
    }
}

class YawSetpoint : public BT::StatefulActionNode
{
    public:
        YawSetpoint(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node); //we use "node_" when we need to 
        // reference the ros node object and "this" when we need to access the class members

        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int first_time_i;
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ar_interfaces::action::Movement>::SharedPtr client_;
        std_msgs::msg::Float32MultiArray setpoint_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bno_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr slider_sub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_pub;

        void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void slider_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        inline float wrapAngle(float angle)
            {
                // Wrap to [0, 2*pi)
                angle = fmod(angle + PI, TWO_PI);
                if (angle < 0.0f)
                    angle += TWO_PI;
            
                // Shift to [-pi, pi)
                return angle - PI;
            }

        float imu_data = 0.0f;
        float turn_ = 0.0f; 
        float port_yaw = 0.0f;
        CurrentYaw current_yaw_ = CurrentYaw::FRONT;

        std::vector<float> slider_data_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; //kp, ki, kd
        long start_time = 0;

        int SET_YAW_BYTE = 0x14;
        int RESET_YAW_BYTE = 0x42;

        float pi;
        const float TWO_PI = 2.0f * M_PI;
        const float PI = M_PI;
};

#endif