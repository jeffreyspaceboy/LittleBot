cmake_minimum_required(VERSION 3.8)
project(lilbot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lilbot_msgs REQUIRED)

add_executable(lilbot_driver_node src/lilbot_driver_node.cpp src/lilbot_pid_controller.cpp src/lilbot_encoder.cpp src/lilbot_motor.cpp)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

if (${ARCHITECTURE} MATCHES "aarch64")
    target_link_libraries(lilbot_driver_node -lpigpio -lrt)
endif()

target_include_directories(lilbot_driver_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
target_compile_features(lilbot_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(lilbot_driver_node rclcpp geometry_msgs lilbot_msgs)

install(TARGETS
	# lilbot_pid_controller
	lilbot_driver_node
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
