#include "tree_flow_nodes/setpoint_loader.hpp"

SetpointLoader::SetpointLoader(const std::string &name,
                               const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config),
      yaml_path_("/yaml_files/setpoints.yaml") {}

BT::PortsList SetpointLoader::providedPorts()
{
  return {
      BT::InputPort<std::string>("color") // "red" or "blue"
  };
}

NodeStatus SetpointLoader::tick()
{
  auto blackboard = config().blackboard;
  auto color = getInput<std::string>("color");

  if (!color)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SetpointLoader"), "Failed to get color input");
    return NodeStatus::FAILURE;
  }

  try
  {
    YAML::Node root = YAML::LoadFile(yaml_path_);
    int count = 0;

    // Top-level keys: "odom", "sick"
    for (auto nav_entry : root)
    {
      std::string nav_type = nav_entry.first.as<std::string>();
      const YAML::Node &nav_node = nav_entry.second;

      // Load color-specific setpoints (red or blue)
      if (nav_node[color.value()])
      {
        loadColorNode(nav_node[color.value()], nav_type, blackboard, count);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("SetpointLoader"),
                    "Color '%s' not found under '%s'",
                    color.value().c_str(), nav_type.c_str());
      }

      // Load common setpoints if present (currently only under odom)
      if (nav_node["common"])
      {
        loadColorNode(nav_node["common"], nav_type, blackboard, count);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("SetpointLoader"),
                "Loaded %d setpoints for color '%s'",
                count, color.value().c_str());
    return NodeStatus::SUCCESS;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SetpointLoader"),
                 "Error loading YAML: %s", e.what());
    return NodeStatus::FAILURE;
  }
}

// Helper: loads all setpoints under a given color/common node into the blackboard
// Keys are prefixed with nav_type (e.g. "odom") and section (e.g. "martial")
// Result: "odom_martial_spear_head", "sick_arena_lock_ramp", etc.
void SetpointLoader::loadColorNode(const YAML::Node &color_node,
                                   const std::string &nav_type,
                                   BT::Blackboard::Ptr blackboard,
                                   int &count)
{
  for (auto section : color_node)
  {
    std::string section_name = section.first.as<std::string>();
    const YAML::Node &section_node = section.second;

    for (auto setpoint : section_node)
    {
      std::string setpoint_name = setpoint.first.as<std::string>();

      // Format: {nav_type}_{section}_{setpoint_name}
      std::string key = nav_type + "_" + section_name + "_" + setpoint_name;

      std::vector<float> values;
      if (setpoint.second.Type() == YAML::NodeType::Sequence)
      {
        for (const auto &val : setpoint.second)
        {
          values.push_back(val.as<float>());
        }
      }
      // Null or anything else stays as empty vector

      blackboard->set(key, values);
      RCLCPP_DEBUG(rclcpp::get_logger("SetpointLoader"),
                   "Set '%s' with %zu values", key.c_str(), values.size());
      count++;
    }
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetpointLoader>("SetpointLoader");
}