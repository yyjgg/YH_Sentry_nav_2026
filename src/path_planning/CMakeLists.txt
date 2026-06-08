cmake_minimum_required(VERSION 3.8)
project(path_planning)

# 默认设置为 Debug 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# 根据构建类型设置编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building in Debug mode with debug symbols")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -DDEBUG")
else()
    message(STATUS "Building in Release mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
endif()

# 通用编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    
    # 在 Debug 模式下添加更多警告
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-Wshadow -Wconversion -Wsign-conversion)
    endif()
endif()

# 检查C++标准
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# 设置 C++ 标准属性
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

# 包含目录
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# 添加调试宏定义
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DEBUG=1)
    message(STATUS "Debug symbols enabled for Valgrind analysis")
endif()

# A*算法库
add_library(astar_lib
  src/AStar.cpp
  src/MapNode.cpp
  src/GridMap.cpp
  src/SafeCorridor.cpp
)

# 为库也设置调试信息
target_compile_options(astar_lib PRIVATE $<$<CONFIG:Debug>:-g -O0>)
target_compile_definitions(astar_lib PRIVATE $<$<CONFIG:Debug>:DEBUG>)

# 链接库的依赖
ament_target_dependencies(astar_lib
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_geometry_msgs 
)

# 主节点
add_executable(path_planning_node
  src/path_planning_node.cpp
  # src/decision_node.cpp
)

# 为主节点也设置调试信息
target_compile_options(path_planning_node PRIVATE $<$<CONFIG:Debug>:-g -O0>)
target_compile_definitions(path_planning_node PRIVATE $<$<CONFIG:Debug>:DEBUG>)

# 决策节点（如果需要独立运行的话）
# add_executable(decision_node
#   src/decision_node.cpp
# )

# 为决策节点也设置调试信息
# target_compile_options(decision_node PRIVATE $<$<CONFIG:Debug>:-g -O0>)
# target_compile_definitions(decision_node PRIVATE $<$<CONFIG:Debug>:DEBUG>)

# 链接依赖
ament_target_dependencies(path_planning_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
  trajectory_msgs
)

target_link_libraries(path_planning_node
  astar_lib
  ${PCL_LIBRARIES}
)

# 为决策节点链接依赖
# ament_target_dependencies(decision_node
#   rclcpp
#   nav_msgs
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   tf2_geometry_msgs
#   visualization_msgs
#   trajectory_msgs
# )

# target_link_libraries(decision_node
#   astar_lib
#   ${PCL_LIBRARIES}
# )

# 安装目标
install(TARGETS
  path_planning_node
  # decision_node
  astar_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装参数文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(astar_lib)
ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
  trajectory_msgs
)

ament_package()