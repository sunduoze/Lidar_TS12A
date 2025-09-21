cmake_minimum_required(VERSION 3.8)
project(delta_2b_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ament_cmake 构建系统
find_package(ament_cmake REQUIRED)

# 查找 ROS2 依赖
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 设置 SDK 路径
#set(delta_lidar_SDK_PATH "./sdk/")

# 获取 SDK 源文件
#FILE(GLOB delta_lidar_SDK_SRC 
#  "${delta_lidar_SDK_PATH}/src/*.cpp"
#)

# 包含目录
#include_directories(
#  ${delta_lidar_SDK_PATH}/include
#  ${delta_lidar_SDK_PATH}/src
#)


# Include directories
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/sdk/include
)

# 添加SDK源文件
set(SDK_SOURCES
    sdk/src/C3iroboticsLidar.cpp
    sdk/src/CArcTime.cpp
    sdk/src/CCountDown.cpp
    sdk/src/CDeviceConnection.cpp
    sdk/src/CLidarPacket.cpp
    sdk/src/CLidarPacketReceiver.cpp
    sdk/src/CLidarUnpacket.cpp
    sdk/src/CSerialConnection.cpp
    sdk/src/CSimulateSerial.cpp
    sdk/src/CTime.cpp
)

# 添加主节点可执行文件
add_executable(delta_2b_lidar_node 
    src/node.cpp
    ${SDK_SOURCES}
)

# 添加客户端节点可执行文件
add_executable(delta_2b_lidar_node_client 
  src/client.cpp
)

# 为目标添加依赖
ament_target_dependencies(
  delta_2b_lidar_node
  rclcpp
  sensor_msgs
)
ament_target_dependencies(
  delta_2b_lidar_node_client
  rclcpp
  sensor_msgs
)

# 安装可执行文件

install(TARGETS
  delta_2b_lidar_node
  delta_2b_lidar_node_client
  DESTINATION lib/${PROJECT_NAME}
)

# 安装其他资源文件（launch、rviz配置等）
install(DIRECTORY 
  launch 
  rviz 
  sdk
  DESTINATION share/${PROJECT_NAME}
)

# 导出包含目录（如果需要被其他包使用）
ament_export_include_directories(include)

# 导出依赖（如果需要被其他包使用）
ament_export_dependencies(
  rclcpp
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

