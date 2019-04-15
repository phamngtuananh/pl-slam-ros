cmake_minimum_required(VERSION 2.8.3)
project(pl_slam_ros)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

# YAML library
find_library(YAML_CPP_LIBRARIES yaml-cpp)
if(NOT YAML_CPP_LIBRARIES)
# If yaml-cpp not found in the system, try finding it as a user CMake-generated project
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
endif(NOT YAML_CPP_LIBRARIES)

# Add catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
)
  
# Add plain cmake packages 
find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system filesystem)	
find_package(Eigen3 REQUIRED)

add_definitions(-std=c++11 -O3)

# Declare catkin package
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES myahrs_driver
  CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp std_msgs sensor_msgs tf
#   DEPENDS
)

# Include dirs
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/dependencies/stvo-pl/3rdparty/line_descriptor/include/
  ${PROJECT_SOURCE_DIR}/dependencies/stvo-pl/include/
  ${PROJECT_SOURCE_DIR}/dependencies/pl-slam/3rdparty/DBoW2/include/
  ${PROJECT_SOURCE_DIR}/dependencies/pl-slam/include/
  ${catkin_INCLUDE_DIRS}
  /usr/include/suitesparse 
)

# List all files (headers) contained by StVO-PL library
# file(GLOB_RECURSE all_include_files RELATIVE "${CMAKE_SOURCE_DIR}" *.h *.hpp)

# Visualize the files of this directory in IDE creating an custom empty target
# add_custom_target(plslam_includes DEPENDS ${all_include_files} SOURCES ${all_include_files})
                                
# Create Executables                      
add_executable(plslam src/main.cpp)

target_link_libraries(plslam 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/dependencies/stvo-pl/3rdparty/line_descriptor/lib/liblinedesc.so
  ${PROJECT_SOURCE_DIR}/dependencies/stvo-pl/lib/libstvo.so
  ${PROJECT_SOURCE_DIR}/dependencies/pl-slam/3rdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/dependencies/pl-slam/lib/libplslam.so
)