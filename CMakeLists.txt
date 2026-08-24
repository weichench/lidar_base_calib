cmake_minimum_required(VERSION 3.8)
project(lidar_base_calib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(lidar_base_calib_node
  src/lidar_base_calib_node.cpp
  src/config.cpp
  src/bag_reader.cpp
  src/ground_calibrator.cpp
  src/imu_estimator.cpp
)

ament_target_dependencies(lidar_base_calib_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
  rosbag2_cpp
  rosbag2_storage
)

target_link_libraries(lidar_base_calib_node
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS lidar_base_calib_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
