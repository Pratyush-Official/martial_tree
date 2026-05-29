#include "movement_nodes/scrollGripperSystem/setHeight.hpp"
using namespace BT;

SetHeight::SetHeight(const std::string& name, const NodeConfiguration& config,
                        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
  node_(node)
    {
        node_ = rclcpp::Node::make_shared("setHeight");

        pub_ = node_->create_publisher<ar_interfaces::msg::GripperCommand>("/gripper_height", 10);

        sub_ = node_->create_subscription<ar_interfaces::msg::GripperStatus>(
            "/gripper_status_height", 10,
            std::bind(&SetHeight::statusCallback, this, std::placeholders::_1));
    }

    BT::PortsList SetHeight::providedPorts()
    {
        return { InputPort<float>("height") };
    }

  
    BT::NodeStatus SetHeight::onStart()
    {
        getInput("height", target_height_);

        ar_interfaces::msg::GripperCommand msg;
        msg.height = target_height_;
        msg.extension = 0;  
        msg.yaw = 0;
        msg.grip_closed = false;

        pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(),"[SetHeight] :: setting height");


        start_time_ = node_->now();
        return NodeStatus::RUNNING;
    }

   
    BT::NodeStatus SetHeight::onRunning() 
    {

      
        
        if (status_.height_reached)
        {
            return NodeStatus::SUCCESS;
        }

        if ((node_->now() - start_time_).seconds() > 10.0)
        {
            return NodeStatus::FAILURE;
        }

        return NodeStatus::RUNNING;
    }

    void SetHeight::onHalted() 
    {
        
    }


    void SetHeight::statusCallback(const ar_interfaces::msg::GripperStatus::SharedPtr msg)
    {
        status_ = *msg;
    }

    
