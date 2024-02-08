cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nusim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###############
## Interface ##
###############

rosidl_generate_interfaces(control_srv
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  control_srv "rosidl_typesupport_cpp")

###########
## Build ##
###########

ament_export_dependencies(
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs
  nuturtlebot_msgs nav_msgs tf2 tf2_geometry_msgs
  nusim
)

set(NUTURTLE_CONTROL_AMENT_TARGET_DEPENDENCIES
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs
  nuturtlebot_msgs nav_msgs tf2 tf2_geometry_msgs
  nusim
)

set(NUTURTLE_CONTROL_TARGET_LINK_LIBRARIES
  yaml-cpp
  turtlelib::turtlelib
  "${cpp_typesupport_target}"
)

# turtle_control node
add_executable(turtle_control src/turtle_control.cpp)
set_target_properties(turtle_control
  PROPERTIES
  OUTPUT_NAME turtle_control
  PREFIX ""
)
ament_target_dependencies(turtle_control
  ${NUTURTLE_CONTROL_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(turtle_control
  ${NUTURTLE_CONTROL_TARGET_LINK_LIBRARIES}
)

# odometry node
add_executable(odometry src/odometry.cpp)
set_target_properties(odometry
  PROPERTIES
  OUTPUT_NAME odometry
  PREFIX ""
)
ament_target_dependencies(odometry
  ${NUTURTLE_CONTROL_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(odometry
  ${NUTURTLE_CONTROL_TARGET_LINK_LIBRARIES}
)

# circle node
add_executable(circle src/circle.cpp)
set_target_properties(circle
  PROPERTIES
  OUTPUT_NAME circle
  PREFIX ""
)
ament_target_dependencies(circle
  ${NUTURTLE_CONTROL_AMENT_TARGET_DEPENDENCIES}
)

target_link_libraries(circle
  ${NUTURTLE_CONTROL_TARGET_LINK_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  turtle_control
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
