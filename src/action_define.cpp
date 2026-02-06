#include "actions/gripper_open.hpp"

class ActionDefine : public BT::SyncActionNode
{
    public:
    ActionDefine(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        odom_subscriber = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_profile,
            std::bind(&ActionDefine::OdometryCallback, this, std::placeholders::_1));

        sick_subscriber = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sick_data", qos_profile,
            std::bind(&ActionDefine::sickCallback, this, std::placeholders::_1));   

        RCLCPP_INFO(node_->get_logger(),"LineFollower::Ready");
        done_flag = false;
    }









}