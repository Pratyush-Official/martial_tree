#include "sick_pid_client.hpp"
#include "yaw_setpoint.hpp"
#include "yaw_pid_client.hpp"
#include "qos_profiles.hpp"
#include "ak_client.hpp"
#include "get_roi_detections.hpp"  
#include "find_hidden_scroll.hpp"
#include "odom_pid_client.hpp"
#include "initialize_ports_actuators.hpp"
#include "wait_for_set.hpp"
#include "zone_change.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

class tree_ticker : public rclcpp::Node
{ 
public:
    explicit tree_ticker(const std::string &node_name);

    void setup();
    void create_behavior_tree();
    void update_behavior_tree();
    void register_custom_action_nodes();

private:
    rclcpp::CallbackGroup::SharedPtr timer_group;
    std::unique_ptr<BT::StdCoutLogger> logger_;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory;
};