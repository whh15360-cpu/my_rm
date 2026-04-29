cmake_minimum_required(VERSION 3.8)
project(armor_pose_solver)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(armor_interfaces REQUIRED)

# 强制 OpenCV 4（包含所有需要的模块）
find_package(OpenCV 4 REQUIRED COMPONENTS
    core
    highgui
    imgproc
    videoio
    imgcodecs
    calib3d
    dnn
)

# 头文件路径
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# 可执行文件（所有子代码）
add_executable(armor_pose_node
    src/Main.cpp                # 主函数
    src/Armors.cpp              # 装甲识别
    src/PoseSolve.cpp           # PnP位姿解算
    src/ExtendedKalman.cpp      # EKF卡尔曼
    src/DrawTrack.cpp           # 画图
    src/Config.cpp              # 配置
    src/Serial.cpp              # 串口
    src/Plotter.cpp             # UDP调试
)

# 链接库
target_link_libraries(armor_pose_node
    ${OpenCV_LIBS}
    #${rclcpp_LIBRARIES}
    #${cv_bridge_LIBRARIES}
    #${sensor_msgs_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    pthread
)

# 此函数自动处理 rclcpp、sensor_msgs 和自定义接口的链接工作
ament_target_dependencies(armor_pose_node
    rclcpp
    sensor_msgs
    cv_bridge
    armor_interfaces # <--- 链接自定义接口库
)
# 安装（ROS2）
install(TARGETS
    armor_pose_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}
)

# ROS2 打包
ament_package()
