cmake_minimum_required(VERSION 3.8)
project(armor_old)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

#  设置头文件目录
include_directories(
  include
)

#  定义可执行文件及其源码
# 包含了所有源文件
add_executable(armor_detector_node
  src/Main.cpp
  src/Yolo.cpp
  #src/Armors.cpp
  src/PoseSolve.cpp
  src/Draw.cpp
  src/Kalman.cpp
  
)

#  链接库

target_link_libraries(armor_detector_node
  rclcpp::rclcpp
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
)

#  宏定义
target_compile_features(armor_detector_node PUBLIC cxx_std_17)

#  安装规则,用于 colcon build 后的部署
install(TARGETS
  armor_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
