#include "qos_profiles.hpp"

#include "movement_nodes/sick_pid_client.hpp"
#include "movement_nodes/go_to_entrance.hpp"
#include "movement_nodes/yaw_setpoint.hpp"
#include "movement_nodes/yaw_pid_client.hpp"
#include "movement_nodes/ak_client.hpp"
#include "movement_nodes/odom_pid_client.hpp"
#include "movement_nodes/climb.hpp"
#include "movement_nodes/roll.hpp"
#include "movement_nodes/vel_publish.hpp"
#include "movement_nodes/brake_test_node.hpp"
#include "movement_nodes/stall_detection.hpp"

#include "movement_nodes/scrollGripperSystem/setHeight.hpp"
#include "movement_nodes/scrollGripperSystem/gripperPose.hpp"
#include "movement_nodes/scrollGripperSystem/gripperAct.hpp"
#include "movement_nodes/scrollGripperSystem/setExtension.hpp"

#include "camera_nodes/get_roi_detections.hpp"  
#include "camera_nodes/find_hidden_scroll.hpp"

#include "actions/vel_until_limit.hpp"
#include "actions/gripper_close.hpp"
#include "actions/gripper_open.hpp"
#include "actions/motor_actuate.hpp"

#include "tree_flow_nodes/initialize_ports_actuators.hpp"
#include "tree_flow_nodes/wait_for_set.hpp"
#include "tree_flow_nodes/zone_change.hpp"
#include "tree_flow_nodes/setpoint_loader.hpp"

#include "tree_flow_nodes/conditions/check_for_C.hpp"
#include "tree_flow_nodes/conditions/check_for_S.hpp"
#include "tree_flow_nodes/conditions/is_inside_forest.hpp"
#include "tree_flow_nodes/conditions/check_for_prox.hpp"
#include "tree_flow_nodes/conditions/always_node.hpp"

#include "tree_flow_nodes/decision_nodes/decision_node.hpp"
#include "tree_flow_nodes/decision_nodes/node_analysis.hpp"
#include "tree_flow_nodes/decision_nodes/path_planner.hpp"
#include "tree_flow_nodes/decision_nodes/entrance_zone_decision.hpp"

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
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

class tree_ticker : public rclcpp::Node
{ 
public:
    explicit tree_ticker(const std::string &node_name);
    void setup();
    void create_behavior_tree();
    void update_behavior_tree();
    void register_custom_action_nodes();
    // --- merged from bt.hpp ---
    void register_condition_nodes();
    void register_decision_nodes();
    void register_actions_nodes();
private:
    rclcpp::CallbackGroup::SharedPtr timer_group;
    std::unique_ptr<BT::StdCoutLogger> logger_;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory;
    std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
    std::unique_ptr<BT::FileLogger2> file_logger_;
};