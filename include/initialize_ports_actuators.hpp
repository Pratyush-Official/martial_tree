#ifndef INITIALIZE_PORTS_ACTUATORS_HPP
#define INITIALIZE_PORTS_ACTUATORS_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/twist.hpp>

enum class FLAG_ALTER_CASE{
    GAME_START,
    SPEAR_ASSEMBLED, //alters "spear_picked" flagand "stage1_roi" flag
    SCROLL_PICKED_FROM_ENTRANCE, //alters "scroll_picked" flag and "store" flag
    CLIMBED_THREE_PHOTOS //alters the "climbed_three_photos" flag and the three "stage_roi" flags and resets in_martial and sets in_forest
    STORE, //alters the "store" flag
    CARRY, //alters the "carry" flag
    RESET_IN_FOREST, //resets "in_forest", sets "in_martial", sets "photo_sequence", resets "climbed_three_photots" flags
    RESET_TO_MARTIAL_FROM_ARENA;
};

class InitializePortsActuators : public BT::SyncActionNode
{
public:
    InitializePortsActuators(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    void flag_alter(FLAG_ALTER_CASE case_);
    void reset_flags();

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

#endif