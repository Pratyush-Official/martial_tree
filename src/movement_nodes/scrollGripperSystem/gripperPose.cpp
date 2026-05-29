#include "movement_nodes/scrollGripperSystem/gripperPose.hpp"


using namespace BT;



GripperPose::GripperPose(const std::string& name, const BT::NodeConfiguration& config,
                            rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
  node_(node)
    {
        node_ = rclcpp::Node::make_shared("gripperPose");

        pub_ = node_->create_publisher<ar_interfaces::msg::GripperCommand>("/gripper_yaw", 10);

        sub_ = node_->create_subscription<ar_interfaces::msg::GripperStatus>(
            "/gripper_status_pose", 10,
            std::bind(&GripperPose::statusCallback, this, std::placeholders::_1));
    }


    BT::PortsList GripperPose::providedPorts()
    {
        return {
          
            BT::InputPort<int>("yaw")
        };
    }

    BT::NodeStatus GripperPose::onStart() 
    {
      
     
        getInput("yaw", y_);

        ar_interfaces::msg::GripperCommand msg;
        msg.height = 0;
        msg.extension = 0;  
        msg.yaw = y_;
        msg.grip_closed = false;
        pub_->publish(msg); 

        RCLCPP_INFO(node_->get_logger(),"[GripperPose] :: posing");


        start_time_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GripperPose::onRunning()
    {
      
        if (status_.yaw_done)
        {
            return BT::NodeStatus::SUCCESS;
        }

        if (timeout())
        {
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
            }

    void GripperPose::onHalted(){}
    void GripperPose::statusCallback(const ar_interfaces::msg::GripperStatus::SharedPtr msg)
    {
        status_ = *msg;
    }
    bool GripperPose::timeout()
    {
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        return std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 3;
    }
        
