cmake_minimum_required(VERSION 3.8)
project(perception_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# 定义自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CleanTask.msg"
  "msg/CleaningTaskList.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# 安装Python节点
install(PROGRAMS
  scripts/perception_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装config文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

