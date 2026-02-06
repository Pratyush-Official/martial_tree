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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ar_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(oakd_roi_detector_interfaces REQUIRED)

set(BEHAVIOUR_SOURCES
  src/go_to_spear.cpp
  src/go_to_forest.cpp
  src/ak_client.cpp
  src/yaw_setpoint.cpp
  src/bt_main.cpp
  src/get_roi_detections.cpp
  src/find_hidden_scroll.cpp
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
  oakd_roi_detector_interfaces
)

target_link_libraries(tree_ticker
  BT::behaviortree_cpp_v3
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

ament_package()
