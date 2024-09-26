cmake_minimum_required(VERSION 3.8)
project(bio_ik2_custom_costs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    Eigen3
    moveit_core
    moveit_ros_planning
    pluginlib
    rclcpp
    tf2_eigen
    tf2_ros
    tf2_kdl
    tf2_geometry_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()


# Include directories
include_directories(
	include/${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIRS}
)

# Add library
set(HEADER_FILES include/${PROJECT_NAME}/custom_goals.hpp)
add_library(custom_goals src/custom_goals.cpp ${HEADER_FILES})

ament_export_targets(custom_goals HAS_LIBRARY_TARGET)

install(
	DIRECTORY include/${PROJECT_NAME}
	DESTINATION include
)

# Link libraries
ament_target_dependencies(
    custom_goals
    PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install executables
install(
	TARGETS custom_goals
	EXPORT custom_goals
    LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

ament_package()
