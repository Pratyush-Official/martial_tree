cmake_minimum_required(VERSION 3.8)
project(martial_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ar_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(oakd_roi_detector_interfaces REQUIRED)
find_package(ar_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(motor_interfaces REQUIRED)

set(BEHAVIOUR_SOURCES
  src/bt_main.cpp

  src/movement_nodes/sick_pid_client.cpp
  src/movement_nodes/ak_client.cpp
  src/movement_nodes/yaw_setpoint.cpp
  src/movement_nodes/odom_pid_client.cpp
  src/movement_nodes/yaw_pid_client.cpp
  src/movement_nodes/go_to_entrance.cpp
  src/movement_nodes/vel_publish.cpp
  src/movement_nodes/roll.cpp
  src/movement_nodes/climb.cpp
  src/movement_nodes/brake_test_node.cpp
  src/movement_nodes/stall_detection.cpp

  src/movement_nodes/scrollGripper/setHeight.cpp
  src/movement_nodes/scrollGripper/gripperPose.cpp
  src/movement_nodes/scrollGripper/gripperAct.cpp
  src/movement_nodes/scrollGripper/setExtension.cpp

  src/camera_nodes/get_roi_detections.cpp
  src/camera_nodes/find_hidden_scroll.cpp

  src/tree_flow_nodes/zone_change.cpp
  src/tree_flow_nodes/wait_for_set.cpp
  src/tree_flow_nodes/initialize_ports_actuators.cpp
  src/tree_flow_nodes/setpoint_loader.cpp 

  src/tree_flow_nodes/conditions/check_for_C.cpp
  src/tree_flow_nodes/conditions/check_for_S.cpp
  src/tree_flow_nodes/conditions/is_inside_forest.cpp
  src/tree_flow_nodes/conditions/check_for_prox.cpp
  src/tree_flow_nodes/conditions/always_node.cpp
  src/tree_flow_nodes/decision_nodes/decision_node.cpp
  src/tree_flow_nodes/decision_nodes/arena_decision_node.cpp
  src/tree_flow_nodes/decision_nodes/node_analysis.cpp
  src/tree_flow_nodes/decision_nodes/path_planner.cpp
  src/tree_flow_nodes/decision_nodes/decision_sick.cpp
  src/tree_flow_nodes/decision_nodes/decision_odom.cpp 

  src/actions/vel_until_limit.cpp
  src/actions/gripper_close.cpp
  src/actions/gripper_open.cpp
  src/actions/motor_actuate.cpp
  
  src/set_destination_from_priority.cpp
  src/get_location_for_stage.cpp
  src/update_current_stage.cpp
  src/check_items_picked.cpp
  include/robotlibpc/cpp/controller/pid.cpp
)

add_executable(tree_ticker ${BEHAVIOUR_SOURCES})

target_include_directories(tree_ticker PRIVATE
  include
)

ament_target_dependencies(tree_ticker
  rclcpp
  rclcpp_action
  ament_index_cpp
  nav_msgs
  std_msgs
  std_srvs
  ar_interfaces
  motor_interfaces
  oakd_roi_detector_interfaces
  geometry_msgs
  yaml-cpp
  behaviortree_cpp
)

target_link_libraries(tree_ticker
  yaml-cpp
)

install(TARGETS tree_ticker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY bt_xml/
  DESTINATION share/${PROJECT_NAME}/bt_xml
)

install(DIRECTORY yaml_files/
    DESTINATION share/${PROJECT_NAME}/yaml_files
)

ament_package()
