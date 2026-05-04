cmake_minimum_required(VERSION 3.5)
project(serial_protocol)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

# 包含头文件目录
include_directories(include)

# 生成可执行文件
add_executable(serial_protocol_node
    src/serial_protocol_node.cpp
)
add_executable(error_protect
    src/error_protect.cpp
)

# 链接依赖
ament_target_dependencies(serial_protocol_node
    rclcpp
    geometry_msgs
    std_srvs
)
ament_target_dependencies(error_protect
    rclcpp
    std_srvs
    nav_msgs
)

# 安装可执行文件
install(TARGETS
    serial_protocol_node
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
    error_protect
    DESTINATION lib/${PROJECT_NAME}
)

# 如果需要安装头文件（可选）
install(DIRECTORY include/
    DESTINATION include
)

ament_package()