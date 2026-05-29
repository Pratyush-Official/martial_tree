#include "movement_nodes/scrollGripperSystem/setExtension.hpp"

using namespace BT;


SetExtension::SetExtension(const std::string& name, const NodeConfiguration& config,
                            rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
  node_(node)
    {
        node_ = rclcpp::Node::make_shared("setExtension");

        pub_ = node_->create_publisher<ar_interfaces::msg::GripperCommand>("/gripper_length", 10);

        sub_ = node_->create_subscription<ar_interfaces::msg::GripperStatus>(
            "/gripper_status_extension", 10,
            std::bind(&SetExtension::statusCallback, this, std::placeholders::_1));
    }

    BT::PortsList SetExtension::providedPorts()
    {
        return { InputPort<float>("length") };
    }

  
    BT::NodeStatus SetExtension::onStart()
    {
        getInput("length", target_length_);

        ar_interfaces::msg::GripperCommand msg;
        msg.height = 0;
        msg.extension = target_length_;  
        msg.yaw = 0;
        msg.grip_closed = false;

        pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(),"[SetExtension] :: setting extension");


        start_time_ = node_->now();
        return NodeStatus::RUNNING;
    }

   
    BT::NodeStatus SetExtension::onRunning() 
    {
      
        if (status_.length_reached)
        {
            return NodeStatus::SUCCESS;
        }

        if ((node_->now() - start_time_).seconds() > 10.0)
        {
            return NodeStatus::FAILURE;
        }

        return NodeStatus::RUNNING;
    }

    void SetExtension::onHalted()
    {
        
    }


    void SetExtension::statusCallback(const ar_interfaces::msg::GripperStatus::SharedPtr msg)
    {
        status_ = *msg;
    }

