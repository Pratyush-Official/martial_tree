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

    const std::string bt_xml =
        ament_index_cpp::get_package_share_directory("martial_tree") +
        "/bt_xml/bt.xml";

    tree_ = factory.createTreeFromFile(bt_xml);

    std::ofstream out("/tmp/bt_node_models.xml");
    out << BT::writeTreeNodesModelXML(factory);
    out.close();
}

void tree_ticker::register_custom_action_nodes()
{
    auto shared_self = std::static_pointer_cast<tree_ticker>(this->shared_from_this());
    factory.registerBuilder<GoToSpear>(
        "GoToSpear",
        [shared_self]
        (const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GoToSpear>(name, config, shared_self);
        }
    );

    factory.registerBuilder<GoToForest>(
        "GoToForest",
        [shared_self]
        (const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GoToForest>(name, config, shared_self);
        }
    );

    factory.registerBuilder<AkClient>(
        "AkClient",
        [shared_self]
        (const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<AkClient>(name, config, shared_self);
        }
    );

    factory.registerBuilder<YawSetpoint>(
        "YawSetpoint",
        [shared_self]
        (const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<YawSetpoint>(name, config, shared_self);
        }
    );

    factory.registerBuilder<GetROIDetections>(
        "GetROIDetections",
        [shared_self]
        (const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GetROIDetections>(name, config, shared_self);
        }
    );

    factory.registerBuilder<FindHiddenScroll>(
        "FindHiddenScroll",
        [](const std::string &name, const BT::NodeConfiguration &config) 
        {
            // Call the constructor with only 2 arguments
            return std::make_unique<FindHiddenScroll>(name, config);
        }
    );

    factory.registerBuilder<OdomClient>(
        "OdomClient",
        [shared_self]
        (const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<OdomClient>(name, config, shared_self);
        }
    );

}

void tree_ticker::update_behavior_tree()
{
    auto status = tree_.tickRoot();

    if (status != BT::NodeStatus::RUNNING)
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
