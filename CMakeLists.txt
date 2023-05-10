cmake_minimum_required(VERSION 3.8)
project(offline_trajectory_publisher_state_machine_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hidro_ros2_utils REQUIRED)
find_package(odri_ros2_interfaces REQUIRED)
find_package(odri_ros2_hardware REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

# Target: talker (executable)
add_executable(talker src/offline_trajectory_publisher_state_machine_interface.cpp)


# ROS2-related libraries
ament_target_dependencies(talker rclcpp)
ament_target_dependencies(talker std_msgs)
ament_target_dependencies(talker hidro_ros2_utils)
ament_target_dependencies(talker odri_ros2_interfaces)
ament_target_dependencies(talker odri_ros2_hardware)

# ament_target_dependencies(talker Eigen3)

# Installation
install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME})
	
install(TARGETS
	talker
	DESTINATION lib/${PROJECT_NAME})
	
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

