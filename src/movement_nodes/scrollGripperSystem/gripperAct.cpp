#include "movement_nodes/scrollGripperSystem/gripperAct.hpp"

using namespace BT;

GripperAct::GripperAct(const std::string& name, const BT::NodeConfiguration& config,
                        rclcpp::Node::SharedPtr node)
      : BT::StatefulActionNode(name, config),
  node_(node)
    {
        node_ = rclcpp::Node::make_shared("gripperAct");

        pub_ = node_->create_publisher<ar_interfaces::msg::GripperCommand>("/gripper_act", 10);

        sub_ = node_->create_subscription<ar_interfaces::msg::GripperStatus>(
            "/gripper_status_act", 10,
            std::bind(&GripperAct::statusCallback, this, std::placeholders::_1));
    }


    BT::PortsList GripperAct::providedPorts()
    {
        return {
            BT::InputPort<bool>("grip"),
           
        };
    }

    BT::NodeStatus GripperAct::onStart() 
    {
      
        getInput("grip", g_);

        ar_interfaces::msg::GripperCommand msg;
        msg.height = 0;
        msg.extension = 0;  
        msg.yaw = 0;
        msg.grip_closed = g_;

        pub_->publish(msg);
      
        RCLCPP_INFO(node_->get_logger(),"[GripperAct] :: acting");


        start_time_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GripperAct::onRunning()
    {
        
      
        if (status_.object_grasped)
        {
            return BT::NodeStatus::SUCCESS;
        }

        if (timeout())
        {
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
            }

    void GripperAct::onHalted() {}

    void GripperAct::statusCallback(const ar_interfaces::msg::GripperStatus::SharedPtr msg)
    {
        status_ = *msg;
    }
    bool GripperAct::timeout()
    {
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        return std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 3;
    }
