#include "movement_nodes/brake_test_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "qos_profiles.hpp"

using namespace BT;

BrakeTest::BrakeTest(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_ptr_(node)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    vel_sub_ = node_ptr_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot_vel", sensor_qos(),
        std::bind(&BrakeTest::vel_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", cmd_vel_qos());

    // 10ms brake loop timer — same rate as production
    brake_timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&BrakeTest::brakeLoop, this));

    RCLCPP_INFO(node_ptr_->get_logger(), "[BrakeTest] Initialized.");
}

// ---------------------------------------------------------------
BT::PortsList BrakeTest::providedPorts()
{
    return {
        BT::InputPort<float> ("move_vx"),
        BT::InputPort<float> ("move_vy"),
        BT::InputPort<int>   ("move_duration_ms"),
        BT::InputPort<double>("brake_gain_x"),
        BT::InputPort<double>("brake_gain_y"),
        BT::InputPort<double>("max_brake_vel"),
        BT::InputPort<int>   ("brake_duration_ms"),
        BT::InputPort<bool>  ("y_direction_flip"),
    };
}

// ---------------------------------------------------------------
BT::NodeStatus BrakeTest::onStart()
{
    // read all params from XML once at start
    move_vx_           = getInput<float> ("move_vx").value_or(0.f);
    move_vy_           = getInput<float> ("move_vy").value_or(0.f);
    move_duration_ms_  = getInput<int>   ("move_duration_ms").value_or(2000);
    brake_gain_x_      = getInput<double>("brake_gain_x").value_or(brake_gain_x_);
    brake_gain_y_      = getInput<double>("brake_gain_y").value_or(brake_gain_y_);
    max_brake_vel_     = getInput<double>("max_brake_vel").value_or(max_brake_vel_);
    brake_duration_ms_ = getInput<int>   ("brake_duration_ms").value_or(brake_duration_ms_);
    y_direction_flip_  = getInput<bool>  ("y_direction_flip").value_or(y_direction_flip_);

    RCLCPP_INFO(node_ptr_->get_logger(),
        "[BrakeTest] onStart — move vx=%.3f vy=%.3f dur=%dms | "
        "gain=(%.2f,%.2f) max_vel=%.2f brake_dur=%dms y_flip=%s",
        move_vx_, move_vy_, move_duration_ms_,
        brake_gain_x_, brake_gain_y_, max_brake_vel_,
        brake_duration_ms_, y_direction_flip_ ? "true" : "false");

    move_start_time_ = node_ptr_->now();
    phase_           = TestPhase::MOVING;
    publishVel(move_vx_, move_vy_);

    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------
BT::NodeStatus BrakeTest::onRunning()
{
    switch (phase_)
    {
        case TestPhase::MOVING:
        {
            auto elapsed_ms = (node_ptr_->now() - move_start_time_).nanoseconds() / 1e6;

            if (elapsed_ms < move_duration_ms_)
            {
                publishVel(move_vx_, move_vy_);
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[BrakeTest] Moving... elapsed=%.0f ms", elapsed_ms);
                return BT::NodeStatus::RUNNING;
            }
            else
            {
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[BrakeTest] Move done. Engaging brake. vel=(%.3f, %.3f)",
                    robot_vel_x_, robot_vel_y_);
                publishVel(0.f, 0.f);
                brake_ticks_left_ = brake_duration_ms_ / 10;
                brake_active_     = true;
                phase_            = TestPhase::BRAKING;
                return BT::NodeStatus::RUNNING;
            }
        }

        case TestPhase::BRAKING:
        {
            if (brake_active_)
            {
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[BrakeTest] Braking... ticks_left=%d", brake_ticks_left_);
                return BT::NodeStatus::RUNNING;
            }
            else
            {
                RCLCPP_INFO(node_ptr_->get_logger(), "[BrakeTest] Test complete. SUCCESS.");
                phase_ = TestPhase::IDLE;
                return BT::NodeStatus::SUCCESS;
            }
        }

        default:
            return BT::NodeStatus::SUCCESS;
    }
}

// ---------------------------------------------------------------
void BrakeTest::onHalted()
{
    brake_active_     = false;
    brake_ticks_left_ = 0;
    phase_            = TestPhase::IDLE;
    publishVel(0.f, 0.f);
    RCLCPP_WARN(node_ptr_->get_logger(), "[BrakeTest] Halted — zeroed vel.");
}

// ---------------------------------------------------------------
void BrakeTest::brakeLoop()
{
    if (!brake_active_) return;

    if (brake_ticks_left_ <= 0)
    {
        brake_active_ = false;
        publishVel(0.f, 0.f);
        RCLCPP_INFO(node_ptr_->get_logger(), "[BrakeTest] Brake done, zeroed.");
        return;
    }

    if (!vel_fresh_) return;

    double y_sign = y_direction_flip_ ? 1.0 : -1.0;

    float vx_cmd = static_cast<float>(-brake_gain_x_ * robot_vel_x_);
    float vy_cmd = static_cast<float>( y_sign * brake_gain_y_ * robot_vel_y_);

    auto clamp = [](float v, float lim) {
        return v >  lim ?  lim :
               v < -lim ? -lim : v;
    };
    vx_cmd = clamp(vx_cmd, static_cast<float>(max_brake_vel_));
    vy_cmd = clamp(vy_cmd, static_cast<float>(max_brake_vel_));

    publishVel(vx_cmd, vy_cmd);

    RCLCPP_INFO(node_ptr_->get_logger(),
        "[BrakeTest] vel=(%.3f,%.3f) cmd=(%.3f,%.3f) ticks_left=%d",
        robot_vel_x_, robot_vel_y_, vx_cmd, vy_cmd, brake_ticks_left_);

    --brake_ticks_left_;
}

// ---------------------------------------------------------------
void BrakeTest::publishVel(float vx, float vy)
{
    geometry_msgs::msg::Twist t;
    t.linear.x  = vx;
    t.linear.y  = vy;
    t.angular.z = 0.0;
    cmd_vel_pub_->publish(t);
}

void BrakeTest::vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) return;
    robot_vel_x_ = msg->data[0];
    robot_vel_y_ = msg->data[1];
    vel_fresh_   = true;
}