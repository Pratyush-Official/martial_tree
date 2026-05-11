#include "movement_nodes/sick_pid_client.hpp"
#include "movement_nodes/go_to_entrance.hpp"
#include "movement_nodes/yaw_setpoint.hpp"
#include "movement_nodes/yaw_pid_client.hpp"
#include "movement_nodes/ak_client.hpp"
#include "movement_nodes/odom_pid_client.hpp"

#include "qos_profiles.hpp"

#include "camera_nodes/get_roi_detections.hpp"  
#include "camera_nodes/find_hidden_scroll.hpp"

#include "tree_flow_nodes/initialize_ports_actuators.hpp"
#include "tree_flow_nodes/wait_for_set.hpp"
#include "tree_flow_nodes/zone_change.hpp"

#include "set_destination_from_priority.hpp"
#include "get_location_for_stage.hpp"
#include "update_current_stage.hpp"
#include "check_items_picked.hpp"
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