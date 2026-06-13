#include "tree_flow_nodes/zone_change.hpp"
using namespace std::placeholders; 

ZoneChange::ZoneChange(const string &name, const BT::NodeConfig &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        set_reset_ = node_->create_subscription<std_msgs::msg::UInt8>("set_reset", 
            qos_profile, 
            std::bind(&ZoneChange::set_reset_subscriber_callback, this, std::placeholders::_1));
        
        // reset_to_martial_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("reset_to_martial", 
        //     qos_profile, y
        //     std::bind(&ZoneChange::reset_to_martial_callback, this, _1));

        RCLCPP_INFO(node_->get_logger(),"ZoneChange initialized");
    }


BT::PortsList ZoneChange::providedPorts(){
    return{
    };
}

void ZoneChange::set_reset_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    set_reset_val_ = msg->data;
    // RCLCPP_INFO(node_->get_logger(), "Button: %d", msg->data);
}

void ZoneChange::reset_to_martial_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    reset_to_martial_ = msg->data;
}

BT::NodeStatus ZoneChange::onStart(){
    return onRunning(); 
}

BT::NodeStatus ZoneChange::onRunning(){

    auto blackboard = config().blackboard;

    (void)blackboard->get("in_martial", in_martial);
    (void)blackboard->get("spear_assembled", spear_assembled);
    (void)blackboard->get("scroll_picked_from_entrance", scroll_picked_from_entrance);
    (void)blackboard->get("climb_position_reached", climb_position_reached);
    (void)blackboard->get("pick_from_entrance", pick_from_entrance);
    (void)blackboard->get("store", store);
    (void)blackboard->get("carry", carry);
    (void)blackboard->get("climbed_three_photos", climbed_three_photos);

    (void)blackboard->get("in_forest", in_forest);
    (void)blackboard->get("photo_sequence", photo_sequence);

    (void)blackboard->get("in_arena", in_arena);
    (void)blackboard->get("nav_to_rack", nav_to_rack);
    (void)blackboard->get("placed_1", placed_1);
    (void)blackboard->get("placed_2", placed_2);
    (void)blackboard->get("color", color_);

    // Contradictory zone flags
    if((in_martial + in_forest + in_arena) > 1){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: multiple zone flags set simultaneously");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    // Scroll picked but spear not assembled
    if(scroll_picked_from_entrance == 1 && spear_assembled == 0){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: scroll picked but spear not assembled");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    // Climbed three photos but scroll not picked
    if(climbed_three_photos == 1 && scroll_picked_from_entrance == 0){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: climbed photos but scroll not picked");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    // In arena but spear not assembled
    if(in_arena == 1 && spear_assembled == 0){
        RCLCPP_ERROR(node_->get_logger(), "ZoneChange: in arena but spear not assembled");
        state_ = "failure";
        return BT::NodeStatus::FAILURE;
    }

    if(set_reset_val_ == 0){

        if(in_martial == 1){
            flag_alter(config().blackboard, FLAG_ALTER_CASE::GAME_START);

            if(color_ == "red"){
                blackboard->set<std::string>("turn", "Right");
            }

            else if(color_ == "blue"){
                blackboard->set<std::string>("turn", "Left");
            }

            state_ = "success";
            return BT::NodeStatus::SUCCESS;
        }

        else if(in_forest == 1){
            flag_alter(config().blackboard, FLAG_ALTER_CASE::RESET_IN_FOREST);

            if(color_ == "red"){
                blackboard->set<std::string>("turn", "Right");
            }

            else if(color_ == "blue"){
                blackboard->set<std::string>("turn", "Left");
            }

            state_ = "success";
            return BT::NodeStatus::SUCCESS;         
        }

        else if(in_arena == 1){
            // if(){
            flag_alter(config().blackboard, FLAG_ALTER_CASE::RESET_IN_ARENA);
            // }

            // else if(){
            //     flag_alter(config().blackboard, FLAG_ALTER_CASE::RESET_TO_MARTIAL_FROM_ARENA);
            // }
            state_ = "success";
            return BT::NodeStatus::SUCCESS;          
        }
    }

    if(set_reset_val_ == 1){

        // No zone flag set when SET is pressed
        if(in_martial == 0 && in_forest == 0 && in_arena == 0){
            RCLCPP_ERROR(node_->get_logger(), "ZoneChange: SET pressed but no zone flag set");
            state_ = "failure";
            return BT::NodeStatus::FAILURE;
        }

        if(state_ == "success") return BT::NodeStatus::SUCCESS;
        if(state_ == "failure") return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING; 
}

void ZoneChange::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "ZoneChange halted");
}