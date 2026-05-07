#ifndef INITIALIZE_PORTS_ACTUATORS_HPP
#define INITIALIZE_PORTS_ACTUATORS_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include "qos_profiles.hpp"

enum class FLAG_ALTER_CASE{
    GAME_START,
    SPEAR_ASSEMBLED, //alters "spear_picked" flagand "stage1_roi" flag
    SCROLL_PICKED_FROM_ENTRANCE, //alters "scroll_picked" flag and "store" flag
    CLIMBED_THREE_PHOTOS, //alters the "climbed_three_photos" flag and the three "stage_roi" flags and resets in_martial and sets in_forest
    STORE, //alters the "store" flag
    CARRY, //alters the "carry" flag
    RESET_IN_FOREST, //resets "in_forest", sets "in_martial", sets "photo_sequence", resets "climbed_three_photots" flags
    RESET_TO_MARTIAL_FROM_ARENA
};
class InitializePortsActuators : public BT::StatefulActionNode
{
public:
    InitializePortsActuators(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void reset_flags(BT::Blackboard::Ptr bb);
    void flag_alter(BT::Blackboard::Ptr bb, const FLAG_ALTER_CASE case_);
private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr team_color_;

    std::string color_;
    void team_color_subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
};

#endif