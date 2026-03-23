#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ar_interfaces/action/movement.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "qos_profiles.hpp"
#include "get_roi_detections.hpp"
#include "yaw_setpoint.hpp"

enum ScrollState
{
    S,
    SAC ,
    C 
};

class Decision : public BT::SyncActionNode{
    public:
    Decision(const std::string &name, const BT::NodeConfiguration &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

    private:
    std::vector<int>pathway;
    std::vector<ScrollState>scroll_states;

    static const std::unordered_map<int,int> forest_height_blue = {
        {1,40},{2,20},{3,40},{4,60},
        {5,40},{6,20},{7,40},{8,60},
        {9,40},{10,20},{11,40},{12,20}
    };

    static const std::unordered_map<int,int> forest_height_red = {
        {1,40},{2,20},{3,40},{4,20},
        {5,40},{6,60},{7,40},{8,60},
        {9,40},{10,20},{11,40},{12,20}
    };

}