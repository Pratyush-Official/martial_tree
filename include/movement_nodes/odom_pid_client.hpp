#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ar_interfaces/action/move_twist.hpp"


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

   
    bool goal_accepted_{false};
    bool goal_rejected_{false};
    bool goal_done_{false};
    bool goal_succeeded_{false};

    float final_avg_dist_{0.0f};

    void reset();
};

