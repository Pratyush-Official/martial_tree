
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "tree_flow_nodes/decision_nodes/decision_node.hpp"
#include "std_msgs/msg/byte.hpp"


using namespace std::placeholders;
// enum  ClimbState
// {
//     UP,
//     DOWN,
//     STOP
// };


class CheckForProx: public BT::SyncActionNode
{
    public:
    CheckForProx(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node
    );
    
        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Byte>> front_subscription_;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Byte>> rear_subscription_;

        bool front_prox_;
        bool rear_prox_;

        void front_subscriber_callback( const std_msgs::msg::Byte::SharedPtr msg);
        void rear_subscriber_callback( const std_msgs::msg::Byte::SharedPtr msg);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
    
    

};
