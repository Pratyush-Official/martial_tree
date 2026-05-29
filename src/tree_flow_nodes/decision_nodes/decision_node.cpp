#include "tree_flow_nodes/decision_nodes/decision_node.hpp"

using namespace std::placeholders;

DecisionNode::DecisionNode(
        const std::string &name,
        const BT::NodeConfig &config,
        const rclcpp::Node::SharedPtr &node
)
:BT::SyncActionNode(name, config),
    node_(node)
{
    node_height = {
        {1,40},{2,20},{3,40},{4,60},
        {5,40},{6,20},{7,40},{8,60},
        {9,40},{10,20},{11,40},{12,20}
    };
    
    rotate_map = 
    {
        {2,  {{5,0},{1,-90},{3,90}}},
        {1,  {{4,0},{2,-90}}},
        {3,  {{6,180},{2,90}}},
        {4,  {{5,-90},{7,180}}},
        {6,  {{5,-90},{9,0}}},
        {9,  {{12,180},{8,-90}}},
        {8,  {{11,180},{9,-90},{7,90}}},
        {11, {{12,-90},{10,90}}},
        {7,  {{10,180},{8,90}}},
        {5,  {{6,-90},{4,-90},{8,0}}},
        {10, {{11,90}}},
        {12, {{11,-90}}}
    };

    // curr_node     = merged_path[0];
    // prev_node     = merged_path[0];
    // next_node     = merged_path[1];
    // counter_ = 2;
    imu_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/imu_data_received", 10,
        std::bind(&DecisionNode::imuCallback, this, std::placeholders::_1));

    inside_ = node->create_publisher<std_msgs::msg::UInt8>("/is_inside_forest", 10);
}
void DecisionNode::imuCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    imu_yaw_      = normalizeAngle(msg->data);
    imu_received_ = true;
}
float DecisionNode::normalizeAngle(float angle)
{
    while (angle >  M_PI) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -M_PI) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}
BT::PortsList DecisionNode::providedPorts()
{
    return{
        BT::InputPort<std::vector<int>>("merged_path"),
        BT::InputPort<std::map<int, int>>("node_state"),
        BT::OutputPort<int>("curr_node"),
        BT::OutputPort<int>("next_node"),
        BT::OutputPort<int>("prev_node"),

        BT::OutputPort<int>("climb_state"),
        BT::OutputPort<int>("next_climb_state"),
        BT::OutputPort<float>("rotate_state"),
        BT::OutputPort<float>("move_x"),
        BT::OutputPort<float>("move_y"),
        BT::OutputPort<int>("curr_node_state"),
        BT::OutputPort<int>("next_node_state")


    };
}

BT::NodeStatus DecisionNode::tick()
{
    auto merged_path = getInput<std::vector<int>>("merged_path").value();
    auto node_state_int  = getInput<std::map<int, int>>("node_state").value();
    for (const auto& [key, val] : node_state_int)
    {
        node_state[key] = static_cast<ScrollState>(val);
    }
   
    if (curr_node == -1)
    {
        prev_node = merged_path[0];  
        curr_node = merged_path[0];   
        next_node = merged_path[1];   
        counter_  = 1;               
   
    }

    if (node_state.find(curr_node) == node_state.end() ||
        node_state.find(next_node) == node_state.end())
    {
        RCLCPP_WARN(node_->get_logger(), "node not in node_state map");
        return BT::NodeStatus::FAILURE;
    }

    ScrollState next_node_state = node_state.at(next_node);
    ScrollState curr_node_state = node_state.at(curr_node);

    climb_calc(curr_node, next_node);
    rotate_state(prev_node, curr_node, next_node, curr_node_state);
    move_xy(prev_node, curr_node, next_node, next_node_state);

    if (counter_ + 1 < (int)merged_path.size())
    {
        int next_next_node = merged_path[counter_ + 1];
        ClimbState next_climb = (node_height[next_next_node] - node_height[next_node] == 20) ? UP : DOWN;
        setOutput<int>("next_climb_state", static_cast<int>(next_climb));
    }
    else
    {
        setOutput<int>("next_climb_state", static_cast<int>(climb_state_));
    }

    setOutput<float>("rotate_state",          rotate_state_);
    setOutput<float>("move_x",                move_x_);
    setOutput<float>("move_y",                move_y_);
    setOutput<int>("climb_state",      static_cast<int>(climb_state_));
    setOutput<int>("prev_node",               prev_node);
    setOutput<int>("curr_node",               curr_node);
    setOutput<int>("next_node",               next_node);
    setOutput<int>("next_node_state", static_cast<int>(next_node_state));
    setOutput<int>("curr_node_state", static_cast<int>(curr_node_state));

    printf("[Decision node]:: prev=%d curr=%d next=%d, rotate_state=%f, move_x=%f, move_y=%f \n", prev_node, curr_node, next_node, rotate_state_, move_x_, move_y_);
    printf("[Decision node]:: climb_state = %i\n",static_cast<int>(climb_state_));
    //    climb_state);

    update_path(merged_path, node_state);  // advance AFTER computing outputs
   
    return BT::NodeStatus::SUCCESS;
}
void DecisionNode::climb_calc(int curr_node, int next_node)
{
    int curr_h = node_height[curr_node];  
    int next_h = node_height[next_node];
    if (next_h - curr_h == -20)
        climb_state_ = DOWN;  
    else
        climb_state_ = UP;
    // int next_next_node = merged_path[counter_ + 1];  
    // ClimbState next_climb = (node_height[next_next_node] - node_height[next_node] == 20) ? UP : DOWN;
    
}
void DecisionNode::rotate_state(int prev_node, int curr_node, int next_node,
                                 ScrollState curr_state)
{
 
    float prev_angle = 0.0;
    float next_angle = 0.0;

    if(curr_state == S)
    {
        prev_angle = rotate_map[prev_node][curr_node];
        next_angle = rotate_map[prev_node][next_node];
    }
    else
    {
        next_angle = rotate_map[curr_node][next_node];
        prev_angle = imu_yaw_ * (180.0f/M_PI);

    }

    rotate_state_ =next_angle - prev_angle;
    printf("rotate: prev=%d curr=%d next=%d state=%d prev_angle=%.1f next_angle=%.1f result=%.1f\n",
        prev_node, curr_node, next_node, (int)curr_state,
        prev_angle, next_angle, rotate_state_);
    
}
void DecisionNode::move_xy(int prev_node, int curr_node, int next_node,
                                 ScrollState curr_state)

{
    
    if(curr_state == S)
    {
        auto [x1, y1]= calculate_xy(prev_node, curr_node);
        auto [x2, y2] = calculate_xy(prev_node, next_node);

        move_x_ = x2 - x1;
        move_y_ = y2 - y1;
    }
    else
    {
        auto [x, y] = calculate_xy(curr_node, next_node);
        move_x_ = x;
        move_y_ = y;
    }
}
std::pair<float,float> DecisionNode::calculate_xy(int curr_node, int next_node)
{
    float robo_length = 680.0;
    float x=0, y=0;

    if(next_node - curr_node == 3)
    {
        x = 500.0 - (robo_length/2);
        y = 0;
    }
    else if(next_node - curr_node == 1)
    {
        x = 400.0 - (robo_length/2);
        y = (400.0 - (robo_length/2));
    }
    else if(next_node - curr_node == -1)
    {
        x = 400.0 - (robo_length/2);
        y = -(400.0 - (robo_length/2));
    }
    else if (next_node - curr_node == 2)
    {
        x = -(400.0 - (robo_length / 2));
        y = 0.0f;
    }

    return {x,y};
}
void DecisionNode::update_path(const std::vector<int>& merged_path,
                                const std::map<int, ScrollState>& node_state)
{
    if (counter_ + 1 >= (int)merged_path.size())
    {
        RCLCPP_INFO(node_->get_logger(), "reached end of path");
        inside.data = 0;
        inside_->publish(inside);
        return;
    }

    prev_node = merged_path[counter_ - 1];   
    curr_node = merged_path[counter_];        
    next_node = merged_path[counter_ + 1];   
    counter_++;                               
    

    inside.data = 1;
    inside_->publish(inside);
}