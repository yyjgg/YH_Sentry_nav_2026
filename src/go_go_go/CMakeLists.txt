cmake_minimum_required(VERSION 3.8)
project(go_go_go)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 尝试多种方式查找OSQP库
find_library(OSQP_LIBRARIES osqp)
find_path(OSQP_INCLUDE_DIRS osqp/osqp.h)

# 如果没找到，尝试更具体的路径
if(NOT OSQP_LIBRARIES OR NOT OSQP_INCLUDE_DIRS)
    find_library(OSQP_LIBRARIES 
        NAMES osqp
        PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
    )
    find_path(OSQP_INCLUDE_DIRS 
        NAMES osqp/osqp.h
        PATHS /usr/local/include /usr/include /usr/include/osqp
    )
endif()

if(NOT OSQP_LIBRARIES)
    message(FATAL_ERROR "OSQP library not found. Please install it with: sudo apt install libosqp-dev")
endif()

if(NOT OSQP_INCLUDE_DIRS)
    message(FATAL_ERROR "OSQP headers not found. Please install the development package.")
endif()

message(STATUS "Found OSQP: ${OSQP_LIBRARIES}")
message(STATUS "OSQP includes: ${OSQP_INCLUDE_DIRS}")

# 添加可执行文件
add_executable(go_go_go_node 
  src/go_go_go_node.cpp
  src/SerialPort.cpp
  #src/MPCController.cpp
)

# 设置包含目录和依赖
target_include_directories(go_go_go_node PUBLIC
  include
  ${Eigen3_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(go_go_go_node
  ${OSQP_LIBRARIES}
  m  # 数学库
)

ament_target_dependencies(go_go_go_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  visualization_msgs
  Eigen3
  
)

install(TARGETS
  go_go_go_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()