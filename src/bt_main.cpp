#include "bt_main.hpp"

using namespace std::chrono_literals;

tree_ticker::tree_ticker(const std::string &node_name)
: rclcpp::Node(node_name)
{
    RCLCPP_INFO(get_logger(), "tree_ticker constructed");
}

void tree_ticker::setup()
{
    create_behavior_tree();
    timer_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = create_wall_timer(
        10ms,
        std::bind(&tree_ticker::update_behavior_tree, this),
        timer_group
    );
}

void tree_ticker::create_behavior_tree()
{
    register_custom_action_nodes();

    auto blackboard = BT::Blackboard::create();

    const std::string pkg = ament_index_cpp::get_package_share_directory("martial_tree");

    // register subtrees first
    factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/climb_three_photos.xml");
    factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/scroll_pick_from_entrance.xml");
    factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/spear_assembly.xml");
    factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/main_bt.xml");//Main tree last

    tree_ = factory.createTree("MainTree", blackboard);

    std::string home = std::getenv("HOME");
    std::ofstream out(home + "/ROBOCON_26/2026_ros2_ws/src/martial_tree/bt_xml/bt_node_models.xml");
    out << BT::writeTreeNodesModelXML(factory);
    out.close();
}

void tree_ticker::register_custom_action_nodes()
{
    auto shared_self = std::static_pointer_cast<tree_ticker>(this->shared_from_this());

    factory.registerBuilder<SickPIDClient>(
        "SickPIDClient",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<SickPIDClient>(name, config, shared_self);
        }
    );

    factory.registerBuilder<YawPIDClient>(
        "YawPIDClient",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<YawPIDClient>(name, config, shared_self);
        }
    );

    factory.registerBuilder<AkClient>(
        "AkClient",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<AkClient>(name, config, shared_self);
        }
    );

    factory.registerBuilder<YawSetpoint>(
        "YawSetpoint",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<YawSetpoint>(name, config, shared_self);
        }
    );

    factory.registerBuilder<GetROIDetections>(
        "GetROIDetections",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<GetROIDetections>(name, config, shared_self);
        }
    );

    factory.registerBuilder<FindHiddenScroll>(
        "FindHiddenScroll",
        [](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<FindHiddenScroll>(name, config);
        }
    );

    factory.registerBuilder<OdomPIDClient>(
        "OdomPIDClient",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<OdomPIDClient>(name, config, shared_self);
        }
    );

    factory.registerBuilder<InitializePortsActuators>(
        "InitializePortsActuators",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<InitializePortsActuators>(name, config, shared_self);
        }
    );

    factory.registerBuilder<WaitForSet>(
        "WaitForSet",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<WaitForSet>(name, config, shared_self);
        }
    );

    factory.registerBuilder<ZoneChange>(
        "ZoneChange",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<ZoneChange>(name, config, shared_self);
        }
    );
}

void tree_ticker::update_behavior_tree()
{
    auto status = tree_.tickWhileRunning();

    if(status != BT::NodeStatus::RUNNING)
    {
        RCLCPP_INFO(
            get_logger(),
            "Behavior Tree finished with status %d",
            static_cast<int>(status)
        );
        timer_->cancel();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tree_ticker>("tree_ticker");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    node->setup();
    executor.spin();

    rclcpp::shutdown();
    return 0;
}