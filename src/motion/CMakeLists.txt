cmake_minimum_required(VERSION 3.16)
project(motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(booster_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoveRequest.msg"
  "msg/HeadRequest.msg"
  "msg/KickRequest.msg"
)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(motion_node src/motion_node.cpp)
target_include_directories(motion_node PRIVATE include)
target_link_libraries(motion_node Eigen3::Eigen ${typesupport_target})
ament_target_dependencies(motion_node rclcpp booster_interface)

install(TARGETS motion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
