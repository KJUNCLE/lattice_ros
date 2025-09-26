cmake_minimum_required(VERSION 3.0.2)
project(lattice_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 创建库
add_library(${PROJECT_NAME}
  src/lattice_planner.cpp
  src/trajectory_generator.cpp
  src/collision_checker.cpp
  src/sensor_fusion.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 创建可执行文件
add_executable(lattice_planner_node src/lattice_planner_node.cpp)
target_link_libraries(lattice_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(sensor_simulator_node src/sensor_simulator_node.cpp)
target_link_libraries(sensor_simulator_node
  ${catkin_LIBRARIES}
)

# 安装
install(TARGETS ${PROJECT_NAME} lattice_planner_node sensor_simulator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)