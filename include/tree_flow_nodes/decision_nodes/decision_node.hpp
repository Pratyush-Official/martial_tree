#ifndef DECISION_NODE_H
#define DECISION_NODE_H

#include <map>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "tree_flow_nodes/decision_nodes/node_analysis.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <std_msgs/msg/float32.hpp>

enum  ClimbState
{
    UP,
    DOWN,
    STOP
};


class  DecisionNode : public BT::SyncActionNode
{
public:
    DecisionNode(
        const std::string &name,
        const BT::NodeConfig &config,
        const rclcpp::Node::SharedPtr &node
    );
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    rclcpp::Node::SharedPtr node_;

    std::map<int, std::map<int, float>> rotate_map;
    std::unordered_map<int,int> node_height;
    std_msgs::msg::UInt8 inside;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr inside_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_;

    ClimbState climb_state_;
    float rotate_state_;
    float move_x_;
    float move_y_;
    
    std::vector<int> merged_path;
    std::map<int, ScrollState> node_state;
   
    int curr_node {-1};
    int prev_node {-1};
    int next_node {-1};
    int counter_  {2};

    std::pair<float,float> calculate_xy(int curr_node, int next_node);
    float imu_yaw_;
    bool imu_received_;
    

    float normalizeAngle(float angle);
    void imuCallback(const std_msgs::msg::Float32::SharedPtr msg);

    void climb_calc(int curr_node, int next_node);
    void rotate_state(int prev_node, int curr_node, int next_node,ScrollState curr_state);
    void move_xy(int prev_node,int curr_node, int next_node,ScrollState scroll_state);
    void update_path(const std::vector<int>& merged_path,const std::map<int, ScrollState>& scroll_state);
};



#endif



 