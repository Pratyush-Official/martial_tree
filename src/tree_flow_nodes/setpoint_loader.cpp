#include "tree_flow_nodes/setpoint_loader.hpp"
#include <yaml-cpp/yaml.h>

SetpointLoader::SetpointLoader(const std::string& name, 
                               const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config),
      yaml_path_("/yaml_files/setpoints.yaml") {}

BT::PortsList SetpointLoader::providedPorts() {
  return {
      BT::InputPort<std::string>("color")  // Input: "red" or "blue"
  };
}

NodeStatus SetpointLoader::tick() {

  auto blackboard = config().blackboard;
  auto color = getInput<std::string>("color");
  if (!color) {
    RCLCPP_ERROR(rclcpp::get_logger("SetpointLoader"), 
                 "Failed to get color input");
    return NodeStatus::FAILURE;
  }

  try {
    YAML::Node config = YAML::LoadFile(yaml_path_);
    
    if (!config[color.value()]) {
      RCLCPP_ERROR(rclcpp::get_logger("SetpointLoader"), 
                   "Color '%s' not found in YAML", color.value().c_str());
      return NodeStatus::FAILURE;
    }

    const YAML::Node& color_node = config[color.value()];
    int count = 0;

    // Iterate through sections (entrance, arena, etc.)
    for (auto section : color_node) {
      std::string section_name = section.first.as<std::string>();
      const YAML::Node& section_node = section.second;

      // Iterate through individual setpoints
      for (auto setpoint : section_node) {
        std::string setpoint_name = setpoint.first.as<std::string>();
        
        // Create key like "red_entrance_spear_head"
        std::string key = section_name + "_" + setpoint_name;
        
        std::vector<float> values;
        
        if (setpoint.second.Type() == YAML::NodeType::Null) {
          // null stays as empty vector
          values = {};
        } else if (setpoint.second.Type() == YAML::NodeType::Sequence) {
          // It's an array like [600.0, 700.0]
          for (const auto& val : setpoint.second) {
            values.push_back(val.as<float>());
          }
        }
        
        // Set on blackboard
        blackboard->set(key, values);
        RCLCPP_DEBUG(rclcpp::get_logger("SetpointLoader"), 
                     "Set %s with %zu values", key.c_str(), values.size());
        count++;
      }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("SetpointLoader"), 
                "Loaded %d setpoints for color '%s'", 
                count, color.value().c_str());
    
    return NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SetpointLoader"), 
                 "Error loading YAML: %s", e.what());
    return NodeStatus::FAILURE;
  }
}

// Register the node
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<SetpointLoader>("SetpointLoader");
}