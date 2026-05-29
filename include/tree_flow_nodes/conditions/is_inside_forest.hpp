#ifndef IS_INSIDE_FOREST_H
#define IS_INSIDE_FOREST_H

#include <string>
#include <memory>
#include <std_msgs/msg/u_int8.hpp>



#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include <behaviortree_cpp/bt_factory.h>



class isInsideForest: public BT::SyncActionNode
{
    public:
    isInsideForest(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;
    
    bool inside;

    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};


#endif