
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"



using namespace std::placeholders;

class AlwaysNode: public BT::SyncActionNode
{
    public:
    AlwaysNode(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node);
    
        rclcpp::Node::SharedPtr node_;
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
    
    

};
