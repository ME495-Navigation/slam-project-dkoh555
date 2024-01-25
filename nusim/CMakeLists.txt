cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

###########
## Build ##
###########

ament_export_dependencies(rclcpp geometry_msgs sensor_msgs)

set(NUSIM_AMENT_TARGET_DEPENDENCIES
  rclcpp geometry_msgs sensor_msgs
)
set(NUSIM_TARGET_LINK_LIBRARIES
  yaml-cpp
  "${cpp_typesupport_target}"
)

# add_executable(nusim src/nusim.cpp)
# target_include_directories(nusim PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(nusim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# nusim node
add_executable(nusim src/nusim.cpp)
set_target_properties(nusim
  PROPERTIES
  OUTPUT_NAME nusim
  PREFIX ""
)
ament_target_dependencies(nusim
  ${NUSIM_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(nusim
  ${NUSIM_TARGET_LINK_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS nusim
  DESTINATION lib/${PROJECT_NAME})

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
