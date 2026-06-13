#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ar_interfaces/action/move_twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "qos_profiles.hpp"


using MoveTwist      = ar_interfaces::action::MoveTwist;
using GoalHandleMove = rclcpp_action::ClientGoalHandle<MoveTwist>;

class OdomPIDClient : public BT::StatefulActionNode
{
public:
    OdomPIDClient(const std::string & name,
               const BT::NodeConfig & config,
               rclcpp::Node::SharedPtr node);

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

    static BT::PortsList providedPorts();


    rclcpp::Node::SharedPtr                     node_;
    rclcpp_action::Client<MoveTwist>::SharedPtr action_client_;
    GoalHandleMove::SharedPtr                   goal_handle_;

    std_msgs::msg::Float32MultiArray setpoint_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; //For immediately stopping after on halted

    int RESET_BYTE = 0x42;

    std::atomic<bool> goal_accepted_{false};
    std::atomic<bool> goal_rejected_{false};
    std::atomic<bool> goal_done_{false};
    std::atomic<bool> goal_succeeded_{false};

    float final_avg_dist_{0.0f};
    float move_x_{0.0f};
    float move_y_{0.0f};
    std::atomic<bool>  goal_sent_{false};

    void reset();
};

