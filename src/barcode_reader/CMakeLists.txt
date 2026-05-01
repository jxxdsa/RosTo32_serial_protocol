cmake_minimum_required(VERSION 3.10)
project(barcode_reader)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 查找 zbar 库
find_library(ZBAR_LIB zbar)

# 可执行文件
add_executable(barcode_reader_node src/barcode_reader_node.cpp)

# 使用 ament_target_dependencies 包含 ROS2 依赖的头文件和库
ament_target_dependencies(barcode_reader_node
  rclcpp
  sensor_msgs
  std_msgs
)

# 添加额外的 OpenCV 和 zbar 链接
target_link_libraries(barcode_reader_node
  ${OpenCV_LIBS}
  ${ZBAR_LIB}
)

# 包含自己的头文件目录 (如果需要 include/ 的头文件)
target_include_directories(barcode_reader_node PRIVATE include)

install(TARGETS barcode_reader_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()