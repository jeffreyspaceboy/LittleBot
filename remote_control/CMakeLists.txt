cmake_minimum_required(VERSION 3.5)
project(remote_control)

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
# ADD PACKAGES TO FIND HERE

# Message Files
set(msg_files
  # ADD MESSAGE FILES HERE, EX: "msg/Num.msg"
)

# Service Files
set(srv_files
  # ADD SERVICE FILES HERE, EX: "srv/AddThreeInts.srv"
)

# Action Files
set(action_files
  # ADD ACTION FILES HERE, EX: "action/RotateAbsolute.action"
)

# Generate Interfaces
# rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
#  ${srv_files}
#  ${action_files}
#  # ADD INTERFACE LISTS HERE
#)

# Add Executables
add_executable(keyboard_control src/keyboard_control.cpp)
ament_target_dependencies(keyboard_control rclcpp rclcpp_action geometry_msgs)
target_include_directories(keyboard_control PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

# Install Targets
install(TARGETS
  keyboard_control
  # ADD NODE NAMES HERE
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights. Uncomment the line when a copyright and license is not present in all source files.
  #set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo). Uncomment the line when this package is not in a git repo.
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
