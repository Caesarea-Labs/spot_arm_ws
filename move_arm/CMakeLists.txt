cmake_minimum_required(VERSION 3.8)
project(move_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(ros2_linkattacher REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(move_arm
    src/move_arm.cpp)
target_include_directories(move_arm
    PUBLIC include)
ament_target_dependencies(move_arm
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	yolov8_msgs)

add_executable(move_arm_class
    src/move_arm_class.cpp)
target_include_directories(move_arm_class
    PUBLIC include)
ament_target_dependencies(move_arm_class
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	yolov8_msgs)



add_executable(test_move
    src/test_move.cpp)
target_include_directories(test_move
    PUBLIC include)
ament_target_dependencies(test_move
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	std_msgs)

add_executable(move_arm_service
    src/move_arm_service.cpp)
target_include_directories(move_arm_service
    PUBLIC include)
ament_target_dependencies(move_arm_service
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	std_msgs
	yolov8_msgs
	moveit_msgs
	tf2
	tf2_ros
	geometry_msgs)

add_executable(cns_task
    src/chip_socket_task.cpp)

target_include_directories(cns_task
    PUBLIC include)
ament_target_dependencies(cns_task
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	std_msgs
	yolov8_msgs
	moveit_msgs
	sensor_msgs
	std_srvs
	tf2
	tf2_ros
	geometry_msgs
	linkattacher_msgs
	ros2_linkattacher)




# Install the executable
install(TARGETS
	move_arm
	test_move
	move_arm_class
	move_arm_service
	cns_task
    DESTINATION lib/${PROJECT_NAME}
)
# Install the launch file
install(DIRECTORY
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
