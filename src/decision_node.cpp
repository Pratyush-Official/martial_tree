#include "decision_node.hpp"

Decision::Decision(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    RCLCPP_INFO(rclcpp::get_logger("GripperClose"),"GripperClose::Ready..");
}

BT::PortsList Decision::providedPorts()
{
    return {
        BT::BidirectionalPort<std::string>("turn", "Direction: Right, Left, Back, Front or raw float"), //Yaw setpoint input
        BT::BidirectionalPort<CurrentYaw>("current_yaw", CurrentYaw::FRONT, "Current facing direction"), //Current yaw
        BT::BidirectionalPort<bool>("up", "Send true to raise the mechanism"), //AK actuation 
        BT::BidirectionalPort<bool>("down", "Send true to lower the mechanism"), //AK actuation 
        BT::BidirectionalPort<int>("idx", 0, "Path planner index"),

        BT::InputPort<std::vector<int>>("path_array", "Gets planned path"), //Planned path arrays
        BT::InputPort<std::vector<ScrollState>>("scroll_array"), //Scroll states array
};   
}

BT::NodeStatus Decision::tick() 
{
    auto path_array_ = getInput<std::vector<int>>("path_array");
    auto scroll_array = getInput<std::vector<ScrollState>>("scroll_array");
    auto idx_ = getInput<int>("idx");

    int  i = idx_.value();
    auto path = path_array_.value();
    std::unordered_map<int, ScrollState> node_state;

    if (!path_array_ || !scroll_array || !idx)
    {
        RCLCPP_WARN(rclcpp::get_logger("Decision"), "Missing port inputs");
        return BT::NodeStatus::FAILURE;
    }

    if (i > static_cast<int>(path.size()) - 2)
    {
        return BT::NodeStatus::FAILURE; 
    } // path exhausted

    for (size_t j = 0; j < path.size(); j++)
    {
        node_state[path[j]] = scrolls[j];
    }

    int curr = path[i];
    int next = path[i + 1];





    RCLCPP_INFO(rclcpp::get_logger("GripperClose"),"Gripper Closed");
    return BT::NodeStatus::SUCCESS;
}