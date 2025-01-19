cmake_minimum_required(VERSION 3.8)
project(px4_ros2_offboard)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add flight_patterns lib
add_library(flight_patterns SHARED src/flight_patterns.cpp)
ament_target_dependencies(flight_patterns geometry_msgs px4_msgs nav_msgs tf2 tf2_geometry_msgs)
target_include_directories(flight_patterns PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# offboard_control_node
add_executable(offboard_control_node src/offboard_control_node.cpp)
target_link_libraries(offboard_control_node flight_patterns)
ament_target_dependencies(offboard_control_node rclcpp px4_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

############
# Install ##
############

install(TARGETS offboard_control_node DESTINATION lib/${PROJECT_NAME})

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

ament_export_targets(export_flight_patterns HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(flight_patterns)

# Install header files
install(DIRECTORY include DESTINATION include)

install(TARGETS flight_patterns
	EXPORT export_flight_patterns
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()