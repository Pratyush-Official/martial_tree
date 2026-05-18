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

    bool debug_mode = this->declare_parameter<bool>("debug", false);
    int tick_ms = debug_mode ? 2000 : 10; //if debug == "true", 2000ms, else 10ms is the tick 

    timer_ = create_wall_timer(
        std::chrono::milliseconds(tick_ms),
        std::bind(&tree_ticker::update_behavior_tree, this),
        timer_group
    );
}

void tree_ticker::create_behavior_tree()
{
    register_custom_action_nodes();

    auto blackboard = BT::Blackboard::create();
    
    blackboard->set<int>("in_martial", 0);
    blackboard->set<int>("spear_assembled", 0);
    blackboard->set<int>("scroll_picked_from_entrance", 0);
    blackboard->set<int>("store",0);
    blackboard->set<int>("carry",0);
    blackboard->set<int>("climbed_three_photos", 0);

    blackboard->set<int>("in_forest", 0);
    blackboard->set<int>("photo_sequence", 0);

    blackboard->set<int>("in_arena",0);
    blackboard->set<int>("placed_1",0);
    blackboard->set<int>("placed_2",0);

    blackboard->set<CurrentYaw>("current_yaw", CurrentYaw::FRONT);

    const std::string pkg = ament_index_cpp::get_package_share_directory("martial_tree");

    try {
        factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/climb_three_photos.xml");
        RCLCPP_INFO(get_logger(), "climb_three_photos OK");
        factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/scroll_pick_from_entrance.xml");
        RCLCPP_INFO(get_logger(), "scroll_pick_from_entrance OK");
        factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/spear_assembly.xml");
        RCLCPP_INFO(get_logger(), "spear_assembly OK");
        factory.registerBehaviorTreeFromFile(pkg + "/bt_xml/main_bt.xml");
        RCLCPP_INFO(get_logger(), "main_bt OK");

        tree_ = factory.createTree("MainTree", blackboard);
        RCLCPP_INFO(get_logger(), "tree created OK");

        logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
        groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, 1667); // default port is 1667
        file_logger_ = std::make_unique<BT::FileLogger2>(tree_, "/home/maomao/ROBOCON_26/2026_ros2_ws/src/martial_tree/bt_xml/bt_trace.btlog");

    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "BT error: %s", e.what());
        throw;
    }

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

    factory.registerBuilder<GoToEntrance>(
        "GoToEntrance",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<GoToEntrance>(name, config, shared_self);
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

    // --- newly added nodes ---

    factory.registerBuilder<SetDestinationFromPriority>(
        "SetDestinationFromPriority",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<SetDestinationFromPriority>(name, config, shared_self);
        }
    );

    factory.registerBuilder<GetLocationForStage>(
        "GetLocationForStage",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<GetLocationForStage>(name, config, shared_self);
        }
    );

    factory.registerBuilder<UpdateCurrentStage>(
        "UpdateCurrentStage",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<UpdateCurrentStage>(name, config, shared_self);
        }
    );

    factory.registerBuilder<CheckItemsPicked>(
        "CheckItemsPicked",
        [shared_self](const std::string &name, const BT::NodeConfig &config)
        {
            return std::make_unique<CheckItemsPicked>(name, config, shared_self);
        }
    );
}

void tree_ticker::update_behavior_tree()
{

    bool debug_mode = this->get_parameter("debug").as_bool();
    auto status = debug_mode ? tree_.tickOnce() : tree_.tickWhileRunning(); //if debug == "true" status = tickOnce, else its tickWhileRunning
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