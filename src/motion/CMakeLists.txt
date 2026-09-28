cmake_minimum_required(VERSION 3.16)
project(motion)

# colcon 的单配置构建未指定类型时默认为空，等价于不优化。
# 机器人上的日常构建统一使用 Release；需要调试时由命令行显式指定。
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(booster_interface REQUIRED)
find_package(booster_msgs REQUIRED)
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
ament_target_dependencies(motion_node ament_index_cpp rclcpp booster_interface booster_msgs)

install(TARGETS motion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config models DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
