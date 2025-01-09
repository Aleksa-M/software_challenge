cmake_minimum_required(VERSION 3.8)
project(problems)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

set(node_plugins "")

# -----------------------------------------------------------------------------------------
# P1
# -----------------------------------------------------------------------------------------

add_library(P1 SHARED src/P1Component.cpp)

ament_target_dependencies(P1
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "std_msgs"
)

rclcpp_components_register_nodes(P1 "composition::P1Component")
set(node_plugins "${node_plugins}composition::P1Component;$<TARGET_FILE:P1>\n")

# -----------------------------------------------------------------------------------------
# P2
# -----------------------------------------------------------------------------------------

add_library(P2 SHARED src/P2Component.cpp)

ament_target_dependencies(P2
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "std_msgs"
  "geometry_msgs"
)

rclcpp_components_register_nodes(P2 "composition::P2Component")
set(node_plugins "{node_plugins}composition::P2Component;$<TARGET_FILE:P2>\n")

# -----------------------------------------------------------------------------------------


# install binary locations
install(TARGETS
  P1
  ARCHIVE DESTINATION lib/
	LIBRARY DESTINATION lib/
	RUNTIME DESTINATION bin/
)

install(TARGETS
  P2
  ARCHIVE DESTINATION lib/
	LIBRARY DESTINATION lib/
	RUNTIME DESTINATION bin/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
