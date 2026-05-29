#ifndef NODE_ANALYSIS_H
#define NODE_ANALYSIS_H

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"


enum ScrollState
{
    S,
    C, 
    SAC
};

class NodeAnalysis : public BT::SyncActionNode
{
public:
    NodeAnalysis(
        const std::string &name,
        const BT::NodeConfig &config,
        const rclcpp::Node::SharedPtr &node
    );
     
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    rclcpp::Node::SharedPtr node_;

    void scroll_path();
    void merge_path();
    int find_nearby_S(int curr_node);

    std::vector<int> planned_path;
    std::vector<int> ar_scrolls;
    std::vector<int> merged_path;

    std::map<int, ScrollState> node_state;

};

#endif

