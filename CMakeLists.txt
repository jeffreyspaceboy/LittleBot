cmake_minimum_required(VERSION 3.5)
project(little_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find Package Dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(little_bot REQUIRED)
# ADD PACKAGES TO FIND HERE

# Message Files
set(msg_files
  "msg/Num.msg"
  # ADD MESSAGE FILES HERE
)

# Service Files
set(srv_files
  "srv/AddThreeInts.srv"
  # ADD SERVICE FILES HERE
)

# Service Files
set(action_files
  "action/RotateAbsolute.action"
  # ADD ACTION FILES HERE
)

# Generate Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  # ADD INTERFACE LISTS HERE
)

# Add Executables
add_executable(remote_controller src/remote_controller.cpp)
ament_target_dependencies(remote_controller rclcpp rclcpp_action geometry_msgs)# little_bot)
target_include_directories(remote_controller PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

add_executable(little_robot src/little_robot.cpp)
ament_target_dependencies(little_robot rclcpp rclcpp_action geometry_msgs) #little_bot)
target_include_directories(little_robot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Install Targets
install(TARGETS
  remote_controller
  little_robot
  # ADD NODE NAME HERE
  DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights. Uncomment the line when a copyright and license is not present in all source files.
  #set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo). Uncomment the line when this package is not in a git repo.
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
