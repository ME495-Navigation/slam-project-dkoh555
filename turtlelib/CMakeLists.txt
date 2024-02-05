cmake_minimum_required(VERSION 3.22) # Set the minimum version of cmake

# State the name of the project
project(turtlelib)

# Create a library
add_library(${PROJECT_NAME} src/geometry2d.cpp src/se2d.cpp
            src/svg.cpp src/diff_drive.cpp)

# Enable C++17 support
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Specify additional compilation flags
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -Wpedantic)

# Include the include directories and their header files
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# frame_main executable
add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)
install(TARGETS frame_main turtlelib
        EXPORT turtlelib-targets)

##############################
## Additional Functionality ##
##############################

# Doxygen
find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" ON)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

# Unit Testing
include(CTest)

# Find the Unit testing framework. In this example, Catch2
find_package(Catch2 3 REQUIRED)

# A test is just an executable that is linked against the unit testing library
# Test for geometry2d.hpp
add_executable(test_geometry2d tests/test_geometry2d.cpp)
target_link_libraries(test_geometry2d ${PROJECT_NAME} Catch2::Catch2WithMain)
# register the test with CTest, telling it what executable to run
add_test(NAME Testing_Geometry2d COMMAND test_geometry2d)

# Test for se2d.cpp
add_executable(test_se2d tests/test_se2d.cpp)
target_link_libraries(test_se2d ${PROJECT_NAME} Catch2::Catch2WithMain)
# register the test with CTest, telling it what executable to run
add_test(NAME Testing_se2d COMMAND test_se2d)

# Test for svg.cpp
add_executable(test_svg tests/test_svg.cpp)
target_link_libraries(test_svg ${PROJECT_NAME} Catch2::Catch2WithMain)
# register the test with CTest, telling it what executable to run
add_test(NAME Testing_svg COMMAND test_svg)

# Test for diff_drive.cpp
add_executable(test_diff_drive tests/test_diff_drive.cpp)
target_link_libraries(test_diff_drive ${PROJECT_NAME} Catch2::Catch2WithMain)
# register the test with CTest, telling it what executable to run
add_test(NAME Testing_diff_drive COMMAND test_diff_drive)

###
### Do I need... any of the below stuff?
###

###########
## Build ##
###########

# Create a library (None rn)
# add_library(${PROJECT_NAME} src/libfile1.cpp src/libfile2.cpp)

# Include the include directories and their header files
# target_include_directories(${PROJECT_NAME}
#     PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
#     $<INSTALL_INTERFACE:include/>)

# Create an executable from the src directory (None rn)
# add_executable(Exec_Name src/Name_main.cpp)

# Add dependencies to the executable (None rn)
# ament_target_dependencies(Exec_Name PUBLIC
#   <dependencies>
# )
# target_link_libraries(Exec_Name
#   PUBLIC
#   ${PROJECT_NAME}
#   PRIVATE
#   cmakeme_flags
# )

#############
## Install ##
#############

# Install include files
# install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install exported targets (None rn)
# install(TARGETS Exec_Name ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets)

# Then export said targets (None rn)
# install(EXPORT ${PROJECT_NAME}-targets
#         FILE ${PROJECT_NAME}-config.cmake
#         NAMESPACE ${PROJECT_NAME}::
#         DESTINATION lib/cmake/${PROJECT_NAME})