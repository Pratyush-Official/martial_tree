#ifndef DECISION_ODOM_HPP_
#define DECISION_ODOM_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp/action_node.h"

#include "camera_nodes/get_roi_detections.hpp"
#include "movement_nodes/yaw_setpoint.hpp"

class DecisionOdom : public BT::SyncActionNode
{
public:
    DecisionOdom(const std::string &name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node);

    ~DecisionOdom() override = default;

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    void set_setpoints(const StageDetections &det, const int yaw_int_,
                       std::string &turn_output, int ran_through);

    void resetVel();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::string under_roi;
    int climb_position_reached{0};
    int pick_from_entrance{0};
    std::vector<float>true_setpoint_;

    std::vector<float> ODOM_SP_MIDDLE;
    std::vector<float> ODOM_SP_NEAR;
    std::vector<float> ODOM_SP_FAR;
};
#endif // DECISION_ODOM_HPP_