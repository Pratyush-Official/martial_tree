#include "tree_flow_nodes/decision_nodes/decision_odom.hpp"

using namespace std::placeholders;
DecisionOdom::DecisionOdom(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node) :                                               // name is passes automatically by the bt_xml, its the name = "" thing in the xml
    BT::SyncActionNode(name, config), node_(node) // config is passed by the bt_xml too for things like ports and shi
{                                                                                                                                                                               // bt nodes arent nodes by themselves, so we passa node to make subscribtions, clients etc accessible

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    RCLCPP_INFO(node_->get_logger(), "DecisionOdom initialized");
}

BT::PortsList DecisionOdom::providedPorts()
{
    return {
        BT::BidirectionalPort<StageDetections>("stage1"),
        BT::InputPort<CurrentYaw>("current_yaw"),
        BT::OutputPort<std::string>("turn", "Direction you wish the robot to rotate: Right, Left, Back, Front"),
        BT::InputPort<std::string>("color", "red or blue"),
        BT::BidirectionalPort<int>("climb_position_reached", "Used to decide wehter the robot as gone to the middle block or not"),
        BT::OutputPort<std::string>("under_roi", "The roi where the robot currently is"),
        BT::OutputPort<int>("pick_from_entrance", "Used to decide wether to pickup scroll from the climbing block or just climb"),
        BT::BidirectionalPort<int>("ran_through", "Used to decide wether the picking/navigation logic has been done before (for reset purposes")
    };
}
BT::NodeStatus DecisionOdom::tick()
{
    under_roi = "roi2";

    auto blackboard = config().blackboard;
    auto scroll_detections = getInput<StageDetections>("stage1");
    auto current_yaw_ = getInput<CurrentYaw>("current_yaw");
    auto color = getInput<std::string>("color");
    auto ran_through = getInput<int>("ran_through").value_or(0);

    (void)blackboard->get("odom_martial_closest_entrance", ODOM_SP_NEAR);
    (void)blackboard->get("odom_martial_middle_entrance", ODOM_SP_MIDDLE);
    (void)blackboard->get("odom_martial_furthest_entrance", ODOM_SP_FAR);

    if (!blackboard->get("odom_martial_closest_entrance", ODOM_SP_NEAR)   ||
    !blackboard->get("odom_martial_middle_entrance",   ODOM_SP_MIDDLE)  ||
    !blackboard->get("odom_martial_furthest_entrance", ODOM_SP_FAR))
    {
        RCLCPP_WARN(node_->get_logger(), "Missing blackboard setpoints, returning FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    if (ODOM_SP_NEAR.empty() || ODOM_SP_FAR.empty() || ODOM_SP_MIDDLE.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Setpoint vectors are empty, returning FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    climb_position_reached = getInput<int>("climb_position_reached").value_or(0);

    if (!scroll_detections)
    {
        RCLCPP_WARN(node_->get_logger(), "No detections on port: %s",
                    scroll_detections.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (!current_yaw_)
    {
        RCLCPP_WARN(node_->get_logger(), "No current_yaw on port");
        return BT::NodeStatus::FAILURE;
    }

    const int yaw_int = static_cast<int>(current_yaw_.value());
    std::string turn_output;
    set_setpoints(scroll_detections.value(), yaw_int, turn_output, ran_through);

    if (!turn_output.empty())
    {
        setOutput<std::string>("turn", turn_output);
    }
    setOutput("climb_position_reached", climb_position_reached);
    setOutput("under_roi", under_roi);
    setOutput("pick_from_entrance", pick_from_entrance);

    blackboard->set("entrance_movement", true_setpoint_);

    return BT::NodeStatus::SUCCESS;
}

void DecisionOdom::set_setpoints(const StageDetections &det, const int yaw_int_,
                                         std::string &turn_output, int ran_through)
{
    if (ran_through || det.roi2 == "real" || (det.roi1 != "real" && det.roi3 != "real"))
    {
        under_roi = "roi2";

        if(det.roi2 == "real"){
            pick_from_entrance = 1;
        }

        switch (yaw_int_)
        {
        // case 0:
        //     true_setpoint = ODOM_SP_FRONT_MIDDLE;
        //     break;
        case 1:
            true_setpoint_ = ODOM_SP_MIDDLE;
            turn_output = "Left";
            break;
        case 3:
            true_setpoint_ = ODOM_SP_MIDDLE;
            turn_output = "Right";
            break;
        }
        climb_position_reached = 1;
        ran_through = 1;
        return;
    }

    if (det.roi1 == "real")
    {
        under_roi = "roi1";
        switch (yaw_int_)
        {
        // case 0:
        //     true_setpoint = ODOM_SP_FRONT_NEAR;
        //     break;
        case 1:
            true_setpoint_ = ODOM_SP_NEAR;
            turn_output = "Left";
            break;
        case 3:
            true_setpoint_ = ODOM_SP_NEAR;
            turn_output = "Right";
            break;
        }

        ran_through = 1;
        return;
    }

    if (det.roi3 == "real")
    {
        under_roi = "roi3";
        switch (yaw_int_)
        {
        // case 0:
        //     true_setpoint = ODOM_SP_FRONT_FAR;
        //     break;
        case 1:
            true_setpoint_ = ODOM_SP_FAR;
            turn_output = "Left";
            break;
        case 3:
            true_setpoint_ = ODOM_SP_FAR;
            turn_output = "Right";
            break;
        }

        ran_through = 1;
        return;
    }
}

// pass turn_output by reference instead of calling setOutput inside
void DecisionOdom::resetVel()
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd_pub_->publish(cmd);
}
