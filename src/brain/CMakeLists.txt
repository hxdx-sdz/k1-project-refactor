cmake_minimum_required(VERSION 3.16)
project(brain)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(booster_interface REQUIRED)
find_package(booster_msgs REQUIRED)
find_package(game_controller_interface REQUIRED)
find_package(vision_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion REQUIRED)
find_package(Threads REQUIRED)

# 与原版 Demo 使用同一套系统 Rerun SDK；不下载、不构建新的 Rerun 依赖。
find_package(rerun_sdk REQUIRED)

# 模块启动配置直接使用 ROS；C++17 用于兼容 ROS2 头文件，业务实现保持朴素写法。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(brain_node src/main.cpp)
target_include_directories(brain_node PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(brain_node Threads::Threads)
ament_target_dependencies(brain_node rclcpp booster_interface booster_msgs game_controller_interface vision_interface geometry_msgs motion)
target_link_libraries(brain_node rerun_sdk)

install(TARGETS brain_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
