cmake_minimum_required(VERSION 3.8)
project(armor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找msg编译所需依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 指定要编译的msg文件
set(msg_files
  "msg/ArmorShoot.msg"
)

# 编译msg，生成C++和Python双接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

# 导出依赖，让其他包能找到这个msg包
ament_export_dependencies(rosidl_default_runtime)

ament_package()

