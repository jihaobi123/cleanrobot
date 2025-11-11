cmake_minimum_required(VERSION 3.8)
project(stm32_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(perception_fusion REQUIRED)

# 安装Python节点
install(PROGRAMS
  scripts/stm32_interface_node.py
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

