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

###########
## Build ##
###########

ament_export_dependencies(
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs nuturtlebot_msgs
)

set(NUTURTLE_CONTROL_AMENT_TARGET_DEPENDENCIES
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs nuturtlebot_msgs
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

#############
## Install ##
#############

install(TARGETS turtle_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  # launch
  # srv
  # config
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