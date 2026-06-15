#include "tree_flow_nodes/decision_nodes/decision_sick.hpp"

DecisionSick::DecisionSick(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "DecisionSick initialized");
}

BT::PortsList DecisionSick::providedPorts()
{
    return {
        BT::BidirectionalPort<StageDetections>("stage1"),
        BT::BidirectionalPort<int>("ran_through"),
        BT::BidirectionalPort<int>("climb_position_reached", "Used to decide wehter the robot as gone to the middle block or not"),
        BT::OutputPort<std::string>("under_roi"),
        BT::OutputPort<int>("pick_from_entrance", "Used to decide wether to pickup scroll from the climbing block or just climb"),
        BT::OutputPort<std::string>("turn", "Direction you wish the robot to rotate: Right, Left, Back, Front"),
        BT::InputPort<CurrentYaw>("current_yaw")
    };
}

BT::NodeStatus DecisionSick::tick()
{
    under_roi = "roi2"; //Default roi
    auto blackboard = config().blackboard;
    auto scroll_detections = getInput<StageDetections>("stage1");
    auto current_yaw_ = getInput<CurrentYaw>("current_yaw");
    auto ran_through = getInput<int>("ran_through").value_or(0);
    auto climb_position_reached = getInput<int>("climb_position_reached").value_or(0);
    
    const int yaw_int = static_cast<int>(current_yaw_.value());
    std::string turn_output;
    if (!scroll_detections)
    {
        RCLCPP_WARN(node_->get_logger(), "No detections on port: %s",
                    scroll_detections.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!blackboard->get("sick_martial_closest_entrance", SICK_SP_NEAR)   ||
        !blackboard->get("sick_martial_middle_entrance",   SICK_SP_MIDDLE)  ||
        !blackboard->get("sick_martial_furthest_entrance", SICK_SP_FAR))
    {
        RCLCPP_WARN(node_->get_logger(), "Missing blackboard setpoints, returning FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    if (SICK_SP_NEAR.empty() || SICK_SP_FAR.empty() || SICK_SP_MIDDLE.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Setpoint vectors are empty, returning FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    true_setpoint_.clear(); 

    set_setpoints(scroll_detections.value(), yaw_int, turn_output, ran_through, climb_position_reached, pick_from_entrance);

    setOutput("climb_position_reached", climb_position_reached);
    setOutput("under_roi", under_roi);
    setOutput("pick_from_entrance", pick_from_entrance);
    setOutput("ran_through", ran_through);

    blackboard->set("entrance_movement_sick", true_setpoint_);

    return BT::NodeStatus::SUCCESS;
}


void DecisionSick::set_setpoints(const StageDetections &det, const int yaw_int_,
                                         std::string &turn_output, int &ran_through, int &climb_position_reached, int &pick_from_entrance)
{
    if (ran_through || det.roi2 == "real" || (det.roi1 != "real" && det.roi3 != "real")) {
        under_roi = "roi2";
        true_setpoint_ = SICK_SP_MIDDLE;

        if (det.roi2 == "real") {
            pick_from_entrance = 1;
        }

        switch (yaw_int_)
        {

        case 0:
            true_setpoint_ = SICK_SP_MIDDLE;
            break;

        case 1:
            true_setpoint_ = SICK_SP_MIDDLE;
            turn_output = "Left";
            break;
        case 3:
            true_setpoint_ = SICK_SP_MIDDLE;
            turn_output = "Right";
            break;
        }
        climb_position_reached = 1;
        ran_through = 1;
        return;
    }
    else if (det.roi1 == "real")
    {
        under_roi = "roi1";
        true_setpoint_ = SICK_SP_NEAR;

        switch (yaw_int_)
        {
        case 0:
            true_setpoint_ = SICK_SP_NEAR;
            break;
        case 1:
            turn_output = "Left";
            break;
        case 3:
            turn_output = "Right";
            break;
        }

        ran_through = 1;
        return;
    }
    else if (det.roi3 == "real")
    {
        under_roi = "roi3";
        true_setpoint_ = SICK_SP_FAR;
        
        switch (yaw_int_)
        {
        case 0:
            break;

        case 1:
            turn_output = "Left";
            break;
        case 3:
            turn_output = "Right";
            break;
        }

        ran_through = 1;
        return;
    }
}