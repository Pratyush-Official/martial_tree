#include "go_to_spear.hpp"
#include "go_to_forest.hpp"
#include "yaw_setpoint.hpp"

#include "qos_profiles.hpp"
#include "ak_client.hpp"
#include "get_roi_detections.hpp"  
#include "find_hidden_scroll.hpp"
#include "odom_client.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

#include <fstream>

class tree_ticker : public rclcpp::Node
{ 
public:
    explicit tree_ticker(const std::string &node_name);

    void setup();
    void create_behavior_tree();
    void update_behavior_tree();

    void register_custom_action_nodes();
    // void register_condition_nodes();
    // void register_control_nodes();
    // void register_decorator_nodes();
    // void register_actionClient_nodes();
private:

    rclcpp::CallbackGroup::SharedPtr timer_group;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory;
};
